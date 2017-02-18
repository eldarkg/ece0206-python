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

''' Main entry point '''


import __init__ as ece0206

from test.test_ram import *
from test.test_so import *
from test.test_si import *


if __name__ == '__main__':
    dev = ece0206.Device()
    dev.open()

    print('serial #', dev.get_serial_number())
    print('PRODUCT_NAME:', dev.get_product_name())
    print('MANUFACTURER_NAME:', dev.get_manufacturer_name())

    print('\nRAM test')
    if not test_ram(dev):
        print('RAM test ERROR')
        dev.close()
        exit()
    print('OK')

    print('\nOutput Channel (SO) Test')
    if not test_so(dev):
        print('OutputChannelTest ERROR')
        dev.close()
        exit()
    print('\r            \r', end='')
    print('OK')

    print('\nInput Channels (SI) Test')
    if not test_si(dev):
        print('InputChannelTest ERROR')
        dev.close()
        exit()
    print('\r            \r', end='')
    print('OK')

    dev.close()
