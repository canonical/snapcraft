#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
snapcraft list-plugins

List the available plugins that handle different types of part.

Usage:
  list-plugins [options]

Options:
  -h --help             show this help message and exit.
"""

import pkgutil

from docopt import docopt

import snapcraft.plugins


def main(argv=None):
    argv = argv if argv else []
    docopt(__doc__, argv=argv)

    for importer, modname, is_package in pkgutil.iter_modules(
            snapcraft.plugins.__path__):
        print(modname.replace('_', '-'))
