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
Test output channel (SO)
'''

import time
import __init__ as ece0206

from test.common import *


_SI_CHANNEL  = 1


def _get_period(so_freq):
    if so_freq == ece0206.SO_FREQ.F12_5KHZ:
        min_period = 2800
        max_period = 2960
    else:
        min_period = int(720 / so_freq - 10)
        max_period = int(720 / so_freq + 10)

    return (min_period, max_period)


def _get_so_array_dim_int(so_array_dim):
    if so_array_dim == 0:
        return RAM_LEN
    else:
        return so_array_dim


def test_so(dev:ece0206.Device) -> bool:
    dev.set_long_mode()

    params = list()
    for i in range(RAM_LEN):
        params.append(get_test_param(i))

    buf256x32_write(dev, params)

    si_parity = ece0206.PARITY.ODD
    so_parity = ece0206.PARITY.ODD
    so_err_en = ece0206.ERR_EN.OUT_32BIT

    print(' SINGLE OUTPUT:')
    so_array_num = 1
    so_delay = 0

    for si_mode in list(ece0206.MODE):
        print_si_mode(si_mode)

        for so_freq in list(ece0206.SO_FREQ):
            si_freq = get_si_frequency(so_freq)
            print_frequency(si_freq, so_freq)

            min_period, max_period = _get_period(so_freq)

            for so_array_dim in (1, 128, 255, 0):
                so_array_dim_int = _get_so_array_dim_int(so_array_dim)

                print('          SO Array Dimension = {0:3d}'\
                      .format(so_array_dim_int))

                dev.si_start(_SI_CHANNEL, si_freq, si_mode, si_parity)
                time.sleep(0.3)
                dev.clear_si_array(_SI_CHANNEL)
                dev.so_start(so_freq, so_array_dim, so_array_num, so_delay,
                             so_err_en, so_parity)
                time.sleep(1.5)

                if not test_input_params(dev, _SI_CHANNEL, so_array_dim_int,
                                         params):
                    dev.si_stop(_SI_CHANNEL)
                    return False

                if not test_period(dev, _SI_CHANNEL, so_array_dim_int,
                                   min_period, max_period):
                    dev.si_stop(_SI_CHANNEL)
                    return False

                dev.si_stop(_SI_CHANNEL)

    for so_array_num in (2, 5):
        print(' MULTIPLE OUTPUT:  SO Array Number = {0:3d}'\
              .format(so_array_num))

        for si_mode in list(ece0206.MODE):
            print_si_mode(si_mode)

            for so_freq in list(ece0206.SO_FREQ):
                si_freq = get_si_frequency(so_freq)
                print_frequency(si_freq, so_freq)

                so_array_dim = 1
                so_delay = 0
                print('          SO Array Dimension = 1')

                dev.si_start(_SI_CHANNEL, si_freq, si_mode, si_parity)
                time.sleep(0.3)
                dev.clear_si_array(_SI_CHANNEL)
                dev.so_start(so_freq, so_array_dim, so_array_num, so_delay,
                             so_err_en, so_parity)

                time.sleep(1)
                input_params = dev.read_array_cc(_SI_CHANNEL)

                array_number = 0
                for input_param in input_params:
                    if (input_param.param & 0x7FFFFFFF) == params[0]:
                        array_number += 1

                if array_number != so_array_num:
                    print('          ERROR: input array number = ',
                          array_number)
                    dev.si_stop(_SI_CHANNEL)
                    return False

                dev.si_stop(_SI_CHANNEL)

                so_array_dim = 0
                print('          SO Array Dimension = 256');
                so_delay = 0
                array_number = 0

                dev.si_start(_SI_CHANNEL, si_freq, si_mode, si_parity)
                time.sleep(0.3)
                dev.clear_si_array(_SI_CHANNEL)
                dev.so_start(so_freq, so_array_dim, so_array_num, so_delay,
                             so_err_en, so_parity)

                start_time = time.time()
                tmp_input_param = dev.read_param_cc(_SI_CHANNEL, 255)
                while ((tmp_input_param.param == 0) and
                       (time.time() - start_time < 1.0)):
                    tmp_input_param = dev.read_param_cc(_SI_CHANNEL, 255)

                if tmp_input_param.param == 0:
                    print('          ERROR: first input array timeout')
                    dev.si_stop(_SI_CHANNEL)
                    return False

                prev_timer = tmp_input_param.timer
                array_number += 1
                start_time = time.time()
                while (time.time() - start_time) < (1.0 * array_number):
                    tmp_input_param = dev.read_param_cc(_SI_CHANNEL, 255)
                    if tmp_input_param.timer != prev_timer:
                        array_number += 1
                        prev_timer = tmp_input_param.timer

                if array_number != so_array_num:
                    print('          ERROR: input array number = ',
                          array_number)
                    dev.si_stop(_SI_CHANNEL)
                    return False

                dev.si_stop(_SI_CHANNEL)

    return True
