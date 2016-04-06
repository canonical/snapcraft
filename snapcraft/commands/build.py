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
snapcraft build

Build parts.

Usage:
  build [options] [PART ...]

Options:
  -h --help             show this help message and exit.

"""

from docopt import docopt

from snapcraft import lifecycle


def main(argv=None, project_options=None):
    argv = argv if argv else []
    args = docopt(__doc__, argv=argv)

    lifecycle.execute('build', args['PART'], project_options)
