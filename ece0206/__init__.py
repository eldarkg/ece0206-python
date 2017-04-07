# Copyright (C) 2017  Eldar Khayrullin <eldar.khayrullin@mail.ru>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

'''
Elcus ECE-0206-1С (ARINC429-USB) device user-space driver.
About device: http://www.elcus.ru/pribors.php?ID=ece-0206-1c
'''

import copy
import select
import threading
import usb1 as usb

from enum import IntEnum, unique

import log


DEV_OLD_VID = 0x04B4
DEV_OLD_PID = 0x8613

DEV_VID = 0x28F0
DEV_PID = 0x0206

DEV_IF = 0

SI_CHS_NUM = 4
SI_BUF_LEN = 1024   #TODO try to use ep desc value

REGISTER_LEN = 4
WORD_LEN = 4
PARAM_LEN = 4
PARAM_ENDIAN = 'little'
ADDR_RANGE = (0, 255)
DELAY_RANGE = (0, 255)
WRITE_ARRAY_RANGE = (1, 127)
SI_CH_RANGE = (1, SI_CHS_NUM)
SI_CH_SET_BUF_MODE = 4
INPUT_BUF_LEN = 256

TIMER_ID = 0x00
LSB_TAG = 0xF


@unique
class EP(IntEnum):
    CMD_SO  = usb.ENDPOINT_OUT | 0x2    # bulk
    ANS     = usb.ENDPOINT_IN | 0x4     # bulk
    SI      = usb.ENDPOINT_IN | 0x6     # interrupt


@unique
class ERR_EN(IntEnum):
    OUT_32BIT = 0
    OUT_33BIT = 1
    OUT_31BIT = 255


@unique
class PARITY(IntEnum):
    SKIP = 0
    ODD = 1


@unique
class SO_FREQ(IntEnum):
    F12_5KHZ = 0
    F50KHZ = 1
    F100KHZ = 2


@unique
class DO_SET(IntEnum):
    NO_CHANGE = 0b00
    TRACK = 0b10
    LVL_1 = 0b01
    LVL_0 = 0b11


@unique
class STATE(IntEnum):
    STOP = 0
    RUN = 1
    ERR = 255 #TODO is need?


@unique
class ERROR_CODE(IntEnum):
    NO = 0x0
    SLOW = 0x8
    LESS32BIT_OR_MORE4T = 0x9
    MORE32BIT_OR_LESS4T = 0xA
    OVERFLOW = 0xB
    PARITY = 0xC


@unique
class SI_FREQ(IntEnum):
    F36TO100KHZ = 0
    F11TO14_5KHZ = 1


@unique
class MODE(IntEnum):
    WORK = 0
    TEST = 1


@unique
class BUF_MODE(IntEnum):
    FULL = 0
    SHORT = 1


@unique
class WORD_TYPE(IntEnum):
    TLABEL = 0
    LSW = 1
    MSW = 2


class SOState:
    def __init__(self):
        self.state = STATE.STOP
        self.err_en = ERR_EN.OUT_32BIT
        self.parity = PARITY.SKIP
        self.freq = SO_FREQ.F12_5KHZ
        self.array_dim = 0
        self.delay = 0                      # LSB 10.24ms
        self.array_num = 0
        self.do_set = DO_SET.NO_CHANGE


class InputParam:
    def __init__(self):
        self.param = 0
        self.timer = 0                      # LSB 4us
        self.error = ERROR_CODE.NO
        self.polling_error = 0 # not used


class ECE0206Exception(Exception): ...


class Device:
    #TODO check input device params range
    def __init__(self, old_id:bool=False) -> None:
        if old_id:
            self._dev_vid = DEV_OLD_VID
            self._dev_pid = DEV_OLD_PID
        else:
            self._dev_vid = DEV_VID
            self._dev_pid = DEV_PID

    def _init_usb_device(self) -> None:
        self._usb_ctx = usb.USBContext()
        self._usb_devh = self._usb_ctx.openByVendorIDAndProductID(self._dev_vid,
                                                                  self._dev_pid)
        if not self._usb_devh:
            raise ECE0206Exception('Device ECE0206 not found.')

        if self._usb_devh.kernelDriverActive(DEV_IF):
            self._usb_devh.detachKernelDriver(DEV_IF)
            self._usb_driver_detached = True

        self._usb_devh.claimInterface(DEV_IF)

    def _init_driver_data(self) -> None:
        self._event_thread_run = True
        self._usb_driver_detached = False
        self._isr = bytearray(SI_CHS_NUM)
        self._timer = 0

        self._msw_words = [bytes(WORD_LEN)] * SI_CHS_NUM
        self._msw_words_timers = [0] * SI_CHS_NUM
        self._buf_ap = [[InputParam()]] * SI_CHS_NUM
        self._buf_cc = [[InputParam()]] * SI_CHS_NUM
        self._buf_cc_i = [0] * SI_CHS_NUM
        for ch in range(SI_CH_RANGE[0], SI_CH_RANGE[1]+1):
            self.clear_si_array(ch)

    def _push_to_buf_cc(self, ch_num:SI_CH_RANGE, iparam:InputParam) -> None:
        self._buf_cc[ch_num-1][self._buf_cc_i[ch_num-1]] = iparam
        if self._buf_cc_i[ch_num-1] < INPUT_BUF_LEN - 1:
            self._buf_cc_i[ch_num-1] += 1
        else:
            self._buf_cc_i[ch_num-1] = 0

    def _process_tlabel_word(self, wrd:bytes) -> None:
        self._timer = 0xFF000000
        self._timer = wrd[2]
        self._timer |= wrd[3] << 8
        self._timer |= wrd[0] << 16
        self._timer &= 0xFFFFFF

    def _get_word_channel(self, wrd:bytes) -> SI_CH_RANGE:
        ch = wrd[1] >> 4
        return ch

    def _process_msw_word(self, wrd:bytes) -> None:
        ch = self._get_word_channel(wrd)
        self._msw_words[ch-1] = wrd
        self._msw_words_timers[ch-1] = self._timer

    def _process_lsw_word(self, wrd:bytes) -> None:
        iparam = InputParam()

        ch = self._get_word_channel(wrd)

        param_bytes = wrd[2:WORD_LEN] + self._msw_words[ch-1][2:WORD_LEN]
        param = int.from_bytes(param_bytes, byteorder='big')
        addr = param & 0xFF
        iparam.param = param

        gta_lsb = self._msw_words[ch-1][0]
        timer = (self._msw_words_timers[ch-1] << 8) | gta_lsb
        iparam.timer = timer

        error_code = self._msw_words[ch-1][1] & 0xF
        iparam.error = ERROR_CODE(error_code)

        self._buf_ap[ch-1][addr] = iparam
        self._push_to_buf_cc(ch, iparam)

        if self._debug:
            self._log.ep_si_param(addr, iparam)

    def _process_si_receive(self, transfer) -> bool:
        def get_type_of_word(word) -> WORD_TYPE:
            if word[1] == TIMER_ID:
                return WORD_TYPE.TLABEL
            elif self._get_word_channel(word) in range(SI_CH_RANGE[0],
                                                       SI_CH_RANGE[1]+1):
                if (word[1] & 0xF) == LSB_TAG:
                    return WORD_TYPE.LSW
                else:
                    return WORD_TYPE.MSW
            else:
                return None

        data = transfer.getBuffer()[:transfer.getActualLength()]

        for i in range(len(data)//WORD_LEN):
            byte_i = WORD_LEN * i
            wrd = bytes(data[byte_i:WORD_LEN+byte_i])

            wrd_type = get_type_of_word(wrd)

            if wrd_type == WORD_TYPE.TLABEL:
                self._process_tlabel_word(wrd)
            elif wrd_type == WORD_TYPE.MSW:
                self._process_msw_word(wrd)
            elif wrd_type == WORD_TYPE.LSW:
                self._process_lsw_word(wrd)

        return True

    def _process_si_errors(self, transfer) -> None:
        #TODO check errors
        ...

    def _start_si_receive(self) -> None:
        def usb_events_thread_func() -> None:
            while self._event_thread_run:
                try:
                    self._usb_ctx.handleEvents()
                except usb.USBErrorInterrupted:
                    pass

        helper = usb.USBTransferHelper()
        helper.setEventCallback(usb.TRANSFER_COMPLETED,
                                self._process_si_receive)
        helper.setDefaultCallback(self._process_si_errors)

        transfer = self._usb_devh.getTransfer()
        transfer.setInterrupt(EP.SI, SI_BUF_LEN, callback=helper)

        self._usb_pthread = threading.Thread(target=usb_events_thread_func)
        self._usb_pthread.start()

        transfer.submit()

    def open(self) -> None:
        '''
        Find and lock usb device.
        On errors raise exceptions.
        '''
        self._log = log.Logger()
        self._log.open('ece0206-log.txt')

        self._init_usb_device()
        self._init_driver_data()
        self._start_si_receive()

    def close(self) -> None:
        '''
        Free the opened usb device.
        '''
        self._event_thread_run = False

        if self._usb_devh:
            if (self._usb_driver_detached and
                not self._usb_devh.kernelDriverActive(DEV_IF)):
                self._usb_devh.attachKernelDriver(DEV_IF)
                self._usb_devh.releaseInterface(DEV_IF)
            self._usb_devh.close()

        if self._usb_pthread:
            self._usb_pthread.join()

        if self._usb_ctx:
            self._usb_ctx.close()

        self._log.close()

    def set_debug(self, level) -> None:
        self._debug = level

    ''' Output channel '''

    def _param_to_bytes(self, param:int) -> bytes(PARAM_LEN):
        return param.to_bytes(PARAM_LEN, byteorder=PARAM_ENDIAN)

    def _bytes_to_param(self, array:bytes(PARAM_LEN)) -> int:
        return int.from_bytes(array, byteorder=PARAM_ENDIAN)

    def _params_to_bytes(self,
            params_array:'list of ints[in WRITE_ARRAY_RANGE]') -> bytearray:
        array = bytearray()

        for param in params_array:
            array.extend(self._param_to_bytes(param))

        return array

    def _check_addr(self, addr:int) -> None:
        if addr not in range(ADDR_RANGE[0], ADDR_RANGE[1]+1):
            raise ECE0206Exception('Attempt to access to wrong address.')

    def _check_write_params_array_dim(self, dim:int) -> None:
        if dim not in range(WRITE_ARRAY_RANGE[0], WRITE_ARRAY_RANGE[1]+1):
            raise ECE0206Exception('Attempt to write long array.')

    def _write_to_ep_cmd(self, data:bytearray) -> None:
        #TODO check len <= 512 bytes
        sent = 0
        while sent < len(data):
            sent += self._usb_devh.bulkWrite(EP.CMD_SO, data[sent:])

        if self._debug:
            self._log.ep_cmd(data)

    def _read_from_ep_ans(self, length) -> bytearray:
        return self._usb_devh.bulkRead(EP.ANS, length)

    def _write_bytes(self, addr:ADDR_RANGE, data:bytearray) -> None:
        data.insert(0, addr)   # AR
        data.insert(1, 0x80)   # CR.Blk

        self._write_to_ep_cmd(data)

    def write_array(self, addr:ADDR_RANGE,
                    params_array:'list of ints[in WRITE_ARRAY_RANGE]') -> None:
        '''
        Write the params array to the SO BUF256x32 buffer address.
        addr: the SO buffer start address;
        params_array: the input params array. Max length WRITE_ARRAY_RANGE.
        '''
        self._check_addr(addr)
        self._check_write_params_array_dim(len(params_array))

        array = self._params_to_bytes(params_array)
        self._write_bytes(addr, array)

    def clear_array(self, addr:ADDR_RANGE, array_dim:WRITE_ARRAY_RANGE) -> None:
        '''
        Clear the SO buffer from set address.
        addr: the SO buffer start address;
        array_dim: the length.
        '''
        self._check_addr(addr)
        self._check_write_params_array_dim(array_dim)

        self._write_bytes(addr, bytearray(array_dim * PARAM_LEN))

    def _pack_osr(self, sostate:SOState) -> bytearray(REGISTER_LEN):
        osr8 = sostate.state << 7                   # OSR.Start/StopOut
        if sostate.err_en != ERR_EN.OUT_32BIT:
            osr8 |= 1 << 6                          # OSR.Err_en
        osr8 |= sostate.do_set << 4                 # OSR.DO_set
        osr8 |= sostate.parity << 3                 # OSR.ParityOut
        if sostate.err_en == ERR_EN.OUT_33BIT:
            osr8 |= 1 << 2                          # OSR.33_31_bit
        osr8 |= sostate.freq                        # OSR.Freq

        osr = bytearray()
        osr.append(sostate.delay)                   # OSR.ODTA
        osr.append(sostate.array_num)               # OSR.Narray
        osr.append(sostate.array_dim)               # OSR.Nwrd
        osr.append(osr8)                            # OSR[8:1]

        return osr

    def _unpack_osr(self, osr:bytearray(REGISTER_LEN)) -> SOState:
        sostate = SOState()

        osr8 = osr[3]
        sostate.state = STATE((osr8 >> 7) & 0b1)    # OSR.Start/StopOut
        err_en = (osr8 >> 6) & 0b1                  # OSR.Err_en
        sostate.parity = PARITY((osr8 >> 3) & 0b1)  # OSR.ParityOut
        bits = (osr8 >> 2) & 0b1                    # OSR.33_31_bit
        sostate.freq = SO_FREQ(osr8 & 0b11)         # OSR.Freq

        if not err_en:
            sostate.err_en = ERR_EN.OUT_32BIT
        elif bits:
            sostate.err_en = ERR_EN.OUT_33BIT
        else:
            sostate.err_en = ERR_EN.OUT_31BIT

        sostate.array_dim = osr[2]                  # OSR.Nwrd
        sostate.array_num = osr[1]                  # OSR.Narray
        sostate.delay = osr[0]                      # OSR.ODTA

        return sostate

    #TODO add _write_osr
    #TODO add _read_osr

    def so_start(self, freq:SO_FREQ, array_dim:ADDR_RANGE, array_num:ADDR_RANGE,
                 delay:DELAY_RANGE=0, err_en:ERR_EN=ERR_EN.OUT_32BIT,
                 parity:PARITY=PARITY.SKIP) -> None:
        '''
        Start to send signals from the SO buffer to the SO.
        freq: the ouput signal frequency;
        array_dim: the output array dimension. 0 => 256;
        array_num: the array output times. 0 - cyclic, 1 - one time;
        delay: the output delay between arrays. LSB 10.24ms. 0 - one time;
        err_en: the output params error length;
        parity: make odd parity bit.
        '''
        sostate = SOState()
        sostate.state = STATE.RUN
        sostate.err_en = err_en
        sostate.parity = parity
        sostate.freq = freq
        sostate.array_dim = array_dim
        sostate.delay = delay
        sostate.array_num = array_num
        sostate.do_set = DO_SET.NO_CHANGE
        data = self._pack_osr(sostate)

        #TODO delete repeated code
        data.insert(0, 0x00)      # AR
        data.insert(1, 0x20)      # CR.Com_buf(register access)

        self._write_to_ep_cmd(data)

    def so_stop(self) -> None:
        '''
        Stop to send signals to the SO.
        '''
        sostate = SOState()
        sostate.state = STATE.STOP
        sostate.err_en = 0
        sostate.parity = 0
        sostate.freq = 0
        sostate.array_dim = 0
        sostate.delay = 0
        sostate.array_num = 0
        sostate.do_set = DO_SET.LVL_0
        data = self._pack_osr(sostate)

        #TODO delete repeated code
        data.insert(0, 0x00)    # AR
        data.insert(1, 0x20)    # CR.Com_buf(register access)

        self._write_to_ep_cmd(data)

    def so_state(self) -> SOState:
        '''
        Survey of the state of the output channel.
        Return: the SO state.
        '''
        data = bytearray()
        #TODO delete repeated code
        data.append(0x00)       # AR
        data.append(0x60)       # CR.Wr_Rd(read) | CR.Com_buf(register access)

        self._write_to_ep_cmd(data)
        osr = self._read_from_ep_ans(REGISTER_LEN)
        if self._debug:
            self._log.ep_ans_osr(osr)

        sostate = self._unpack_osr(osr)

        return sostate

    def write_param(self, addr:ADDR_RANGE, param:int, comm=0) -> None:
        '''
        Write the parameter to SO buffer address.
        addr: the SO buffer address;
        param: the input parameter.
        '''
        data = bytearray()
        #TODO delete repeated code
        data.append(addr)   # AR
        data.append(comm)   # CR
        data.extend(self._param_to_bytes(param))

        self._write_to_ep_cmd(data)

    def read_param(self, addr:ADDR_RANGE) -> int:
        '''
        Read the parameter from SO buffer address.
        addr: the SO buffer address.
        Return: the read parameter.
        '''
        data = bytearray()
        #TODO delete repeated code
        data.append(addr)   # AR
        data.append(0x40)   # CR.Wr_Rd(read)

        self._write_to_ep_cmd(data)
        param = self._read_from_ep_ans(PARAM_LEN)
        if self._debug:
            self._log.ep_ans_param(addr, param)

        return self._bytes_to_param(param)

    ''' Input channels '''

    def _pack_isrb(self, state:STATE, mode:MODE, parity:PARITY,
                   freq:SI_FREQ) -> 'byte':
        isrb = state << 7           # Start/StopIN
        isrb |= mode << 4           # TestMode
        isrb |= parity << 3         # ParityIN
        isrb |= freq                # Fast/Slow

        return isrb

    def _set_isrb_buf_mode(self, isrb:'byte', buf_mode:BUF_MODE) -> 'byte':
        isrb |= buf_mode << 1       # Full/Short

        return isrb

    def _write_isrb(self, ch_num:SI_CH_RANGE, state:STATE, freq:SI_FREQ,
                    mode:MODE, parity:PARITY) -> None:
        data = bytearray()
        #TODO delete repeated code
        data.append(0x00)       # AR
        data.append(0x24)       # CR.Com_buf(register access) | CR.Ch_adr(ISR)

        isrb = self._pack_isrb(state, mode, parity, freq)
        if ch_num == SI_CH_SET_BUF_MODE:
            isrb = self._set_isrb_buf_mode(isrb, self._buf_mode)
        self._isr[ch_num-1] = isrb

        data.extend(self._isr)

        self._write_to_ep_cmd(data)

    def si_start(self, ch_num:SI_CH_RANGE, freq:SI_FREQ, mode:MODE=MODE.WORK,
                 parity:PARITY=PARITY.SKIP) -> None:
        '''
        Start to receive signals from the SI channel to the SI buffers.
        ch_num: the input channel number;
        freq: the input signal frequency range;
        mode: the mode of work of the device;
        parity: check odd parity bit.
        '''
        self._write_isrb(ch_num, STATE.RUN, freq, mode, parity)

    def si_stop(self, ch_num:SI_CH_RANGE) -> None:
        '''
        Stop to receive signals from the SI channel to the SI buffers.
        ch_num: the input channel number.
        '''
        self._write_isrb(ch_num, STATE.STOP, 0, 0, 0)

    def clear_si_array(self, ch_num:SI_CH_RANGE) -> None:
        '''
        Clear receive the SI buffers.
        ch_num: the input channel number.
        '''
        self._buf_ap[ch_num-1] = [InputParam()] * INPUT_BUF_LEN
        self._buf_cc[ch_num-1] = [InputParam()] * INPUT_BUF_LEN
        self._buf_cc_i[ch_num-1] = 0

    def read_array_ap(self,
            ch_num:SI_CH_RANGE) -> 'list of InputParam [INPUT_BUF_LEN]':
        '''
        Read the SI address parameter buffer of input channel.
        ch_num: the input channel number.
        '''
        return copy.copy(self._buf_ap[ch_num-1])

    def read_array_cc(self,
            ch_num:SI_CH_RANGE) -> 'list of InputParam [INPUT_BUF_LEN]':
        '''
        Read the SI ring buffer of input channel.
        ch_num: the input channel number.
        '''
        return copy.copy(self._buf_cc[ch_num-1])

    def read_param_ap(self, ch_num:SI_CH_RANGE,
                      idx:(0, INPUT_BUF_LEN-1)) -> InputParam:
        '''
        Read the parameter from the SI address parameter buffer.
        ch_num: the input channel number;
        idx: the index of parameter in the buffer (idx => addr).
        '''
        return self._buf_ap[ch_num-1][idx]

    def read_param_cc(self, ch_num:SI_CH_RANGE,
                      idx:(0, INPUT_BUF_LEN-1)) -> InputParam:
        '''
        Read the parameter from the SI ring buffer.
        ch_num: the input channel number;
        idx: the index of parameter in the buffer.
        '''
        return self._buf_cc[ch_num-1][idx]

    ''' One cmd '''

    def do_clr(self):
        ...

    def do_set(self):
        ...

    def do_sprt(self):
        ...

    ''' Service call '''

    def get_timer(self) -> int:
        '''
        Get the GTA timer value. LSB 1024us.
        '''
        return self._timer

    def _apply_buf_mode(self, buf_mode:BUF_MODE) -> None:
        self._buf_mode = buf_mode
        isrb = self._pack_isrb(STATE.STOP, 0, 0, 0)
        isrb = self._set_isrb_buf_mode(isrb, self._buf_mode)
        self._isr[SI_CH_SET_BUF_MODE-1] = isrb

    def set_long_mode(self) -> None:
        '''
        Set of the SI buffer sending mode. Send the buffer if it is full.
        Must be called the one time of before the start of input channels.
        '''
        self._apply_buf_mode(BUF_MODE.FULL)

    def set_short_mode(self) -> None:
        '''
        Set of the SI buffer sending mode. Send the buffer each 10.24ms.
        Must be called the one time of before the start of input channels.
        '''
        self._apply_buf_mode(BUF_MODE.SHORT)

    def get_manufacturer_name(self) -> str:
        '''
        Get the device manufacturer name
        '''
        device = self._usb_devh.getDevice()

        return device.getManufacturer()

    def get_product_name(self) -> str:
        '''
        Get the device product name
        '''
        device = self._usb_devh.getDevice()

        return device.getProduct()

    def get_serial_number(self) -> str:
        '''
        Get the device serial number
        '''
        device = self._usb_devh.getDevice()

        return device.getSerialNumber()
