# Copyright (C) 2017  Eldar Khayrullin <eldar.khayrullin@mail.ru>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

'''
Test common functions
'''

import time
import __init__ as ece0206


RAM_LEN             = ece0206.ADDR_RANGE[1] + 1

_BUF_WRITE_BLOCK    = 64
_WRITE_TIMEOUT      = 0.3 # s


def buf256x32_write(dev:ece0206.Device, params):
    print('BUF256x32_write')
    for i in range(4):
        buf_i = i * _BUF_WRITE_BLOCK
        dev.write_array(buf_i, params[buf_i:buf_i + _BUF_WRITE_BLOCK])
        time.sleep(_WRITE_TIMEOUT)


def print_frequency(si_freq, so_freq):
    if si_freq == ece0206.SI_FREQ.F11TO14_5KHZ:
        print('      frequency: si = Slow', end='')
    else:
        print('      frequency: si = Fast', end='')

    if so_freq == ece0206.SO_FREQ.F12_5KHZ:
        print(' so = 12,5KHz')
    elif so_freq == ece0206.SO_FREQ.F50KHZ:
        print(' so = 50KHz')
    elif so_freq == ece0206.SO_FREQ.F100KHZ:
        print(' so = 100KHz')


def test_input_params(dev, si_channel, so_array_dim_int, params):
    input_params = dev.read_array_cc(si_channel)

    for i in range(so_array_dim_int):
        if ((params[i] != (input_params[i].param & 0x7FFFFFFF) or
            input_params[i].error != ece0206.ERROR_CODE.NO) and
            not input_params[i].polling_error):
            print('ERROR: parameter#: 0x{0:x}  right value: 0x{1:x}  '\
                  'value: 0x{2:x}  error: 0x{3:02x}  '\
                  'polling error: 0x{4:02x}'\
                  .format(i, params[i], input_params[i].param,
                          input_params[i].error,
                          input_params[i].polling_error))
            return False

    return True


def test_period(dev, si_channel, so_array_dim_int, min_period, max_period):
    input_params = dev.read_array_cc(si_channel)

    for i in range(so_array_dim_int - 1):
        t0 = input_params[i].timer * 4
        t1 = input_params[i+1].timer * 4
        dt = t1 - t0

        if (dt < min_period) or (dt > max_period):
            print('ERROR: parameter#: 0x{0:x}   '\
                  'input parameter period = {1:d}  '\
                  'Period template = {2:d} - {3:d}'\
                  .format(i, dt, min_period, max_period))
            return False

    return True


def get_si_frequency(so_freq):
    if so_freq == ece0206.SO_FREQ.F12_5KHZ:
        si_freq = ece0206.SI_FREQ.F11TO14_5KHZ
    else:
        si_freq = ece0206.SI_FREQ.F36TO100KHZ

    return si_freq


def print_si_mode(si_mode):
    if si_mode == ece0206.MODE.TEST:
        print('   Self-checking mode')
    elif si_mode == ece0206.MODE.WORK:
        print('   Operating mode (with stub)')


def get_test_param(n):
    return 0x70000000 + (n<<16) + (((~n)<<8) & 0xff00) + n
