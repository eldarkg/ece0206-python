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
Test RAM
'''

import __init__ as ece0206

from test.common import *


_RAM_LEN        = ece0206.ADDR_RANGE[1] + 1
_TEST_VALUE_0   = 0x55555555
_TEST_VALUE_1   = 0xAAAAAAAA


def _is_odd(num:int) -> bool:
    return num & 1


def _get_test_param(cycle:int, idx:int) -> int:
    if cycle == 1:
        if _is_odd(idx):
            return _TEST_VALUE_0
        else:
            return _TEST_VALUE_1
    else:
        if _is_odd(idx):
            return _TEST_VALUE_1
        else:
            return _TEST_VALUE_0


def _print_error(cycle, addr, ram_param, templ_param):
    print('error: cycle#={0} parameter#=0x{1:X} '\
          'value=0x{2:X} (right value=0x{3:X})'.format(cycle, addr,
                                                       ram_param, templ_param))


def _test_ram_cycle(dev:ece0206.Device, cycle:int) -> bool:
    param = list()

    for i in range(_RAM_LEN):
        param.append(_get_test_param(cycle, i))

    buf256x32_write(dev, param)

    for i in range(_RAM_LEN):
        templ = _get_test_param(cycle, i)
        ram_param = dev.read_param(i)

        if ram_param != templ:
            _print_error(1, i, ram_param, templ)
            return False

    return True


def test_ram(dev:ece0206.Device) -> bool:
    if not _test_ram_cycle(dev, 1):
        return False

    return _test_ram_cycle(dev, 2)
