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
Test input channels (SI)
'''

import time

from ..device import *
from .common import *


def test_si(dev:Device) -> bool:
    params = list()
    for i in range(RAM_LEN):
        params.append(get_test_param(i))

    buf256x32_write(dev, params)

    so_err_en = ERR_EN.OUT_32BIT
    so_parity = PARITY.ODD
    so_array_dim = 0
    so_delay = 0
    so_array_num = 0
    si_parity = PARITY.ODD

    dev.set_short_mode()

    for si_ch_num in range(1, SI_CHS_NUM+1):
        print('Channel # ', si_ch_num);

        for si_mode in list(MODE):
            print_si_mode(si_mode)

            for so_freq in list(SO_FREQ):
                si_freq = get_si_frequency(so_freq)
                print_frequency(si_freq, so_freq)

                for si_ch_num_1 in range(1, SI_CHS_NUM+1):
                    dev.si_start(si_ch_num_1, si_freq, si_mode, si_parity)
                    time.sleep(0.1)
                    dev.clear_si_array(si_ch_num_1)

                dev.so_start(so_freq, so_array_dim, so_array_num, so_delay,
                             so_err_en, so_parity)
                time.sleep(1)

                if not test_input_params(dev, si_ch_num, RAM_LEN, params):
                    dev.so_stop()
                    for ch in range(1, SI_CHS_NUM+1):
                        dev.si_stop(ch)

                    return False

                dev.so_stop()
                for ch in range(1, SI_CHS_NUM+1):
                    dev.si_stop(ch)

    return True
