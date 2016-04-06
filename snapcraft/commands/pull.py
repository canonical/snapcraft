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
snapcraft pull

Download or retrieve artifacts defined for a part.

Usage:
  pull [options] [PART ...]

Options:
  -h --help             show this help message and exit.
  --enable-geoip        enables geoip for the pull step if stage-packages
                        are used.

"""

from docopt import docopt

import snapcraft
from snapcraft import lifecycle


def main(argv=None):
    argv = argv if argv else []
    args = docopt(__doc__, argv=argv)

    project_options = snapcraft.get_project_options()
    if args['--enable-geoip']:
        project_options.use_geoip = True

    lifecycle.execute('pull', args['PART'])
