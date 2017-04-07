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


import time

from __init__ import *


class Logger:
    def open(self, name):
        self._log = open(name, 'a')
        self._start_time = time.perf_counter()

    def close(self):
        self._log.close()

    def _write(self, msg):
        msg = str.format('{0:.3f} ' + msg + '\n',
                         time.perf_counter() - self._start_time)
        self._log.write(msg)

    def _param_str(self, addr, param):
        paddr = param & 0xFF
        pid = (param >> 8) & 0x3
        pdata = (param >> 10) & 0x7FFFF
        pms = (param >> 29) & 0x3
        pparity = (param >> 31) & 0x1

        return str.format('0x{0:02X} o{1:03o} 0b{2:02b} '\
                          '0x{3:07X} 0b{4:02b} {5:d}',
                          addr, paddr, pid, pdata, pms, pparity)

    def ep_ans_osr(self, data):
        osr = int.from_bytes(data, 'little')
        self._write(str.format('IN SO OSR 0x{0:08X}', osr))

    def ep_ans_param(self, addr, data):
        param = int.from_bytes(data, 'little')
        self._write(str.format('IN SO PAR ' + self._param_str(addr, param)))

    def ep_cmd(self, data):
        access = (data[1] >> 6) & 0x1
        if access:
            msg = 'OR'
        else:
            msg = 'OW'

        msg += ' '

        ch = data[1] & 0xF
        if ch == 0:
            msg += 'SO'
        elif ch == 4:
            msg += 'SI'
        else:
            msg += 'UN'

        msg += ' '
        blk = (data[1] >> 7) & 0x1
        dest = (data[1] >> 5) & 0x1
        if dest:
            if ch == 0:
                msg += 'OSR'
            elif ch == 4:
                msg += 'ISR'
            reg = int.from_bytes(data[2:], 'little')
            self._write(msg + str.format(' 0x{0:08X}', reg))
        else:
            addr = data[0]
            smsg = msg
            if blk:
                smsg += str.format('BLK 0x{0:02X} {1} ', addr, len(data))
            if access:
                if not blk:
                    smsg += str.format('PAR 0x{0:02X}', addr)
                self._write(smsg)
                return

            self._write(smsg + 'BEG')

            msg += 'PAR '
            wrdlen = 4
            i = 2
            while i < len(data):
                param = int.from_bytes(data[i:i+wrdlen], 'little')
                self._write(msg + self._param_str(addr, param))
                addr += 1
                i += wrdlen

            self._write(smsg + 'END')

    def ep_si_param(self, addr, iparam):
        self._write(str.format('IN SI PAR ' +
                               self._param_str(addr, iparam.param) +
                               ' {0} {1}', iparam.timer, iparam.error))
