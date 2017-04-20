# -*- coding: utf-8 -*-
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
Setup
'''


try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

setup(name = 'ece0206-python',
      version = '0.1',
      author = 'Eldar Khayrullin',
      author_email = 'eldar.khayrullin@mail.ru',
      url = 'https://github.com/eldarkg/ece0206-python',
      description = 'ECE-0206-1С (ARINC429-USB) Python API',
      license = 'LGPLv3',
      platforms = 'any',
      packages = ['ece0206', 'ece0206.test'])
