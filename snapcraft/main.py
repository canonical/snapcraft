#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
snapcraft

Usage:
  snapcraft [--version | --help] [options] COMMAND [ARGS ...]

Options:
  -h --help        show this help message and exit
  -v --version     show program version and exit
  -V --verbose     print additional information about command execution

The available commands are:
  list-parts   List available parts which are like “source packages” for snaps.
  list-plugins List the available plugins that handle different types of part.
  init         Initialize a snapcraft project.
  add-part     Add a part to your snapcraft.yaml, interactively presenting
               options.
  help         Obtain help for a certain plugin or topic
  login        Authenticate session against Ubuntu One SSO.
  logout       Clear session credentials.

The available lifecycle commands are:
  clean        Remove content - cleans downloads, builds or install artifacts.
  cleanbuild   Create a snap using a clean environment managed by lxd.
  pull         Download or retrieve artifacts defined for a part.
  build        Build artifacts defined for a part.
  stage        Stage the part's built artifacts into the common staging area.
  strip        Final copy and preparation for the snap.
  snap         Create a snap.
  upload       Upload a snap to the Ubuntu Store.

See 'snapcraft COMMAND --help' for more information on a specific command.

For more help, visit the documentation:
http://developer.ubuntu.com/snappy/snapcraft
"""

import pkg_resources
import sys
import textwrap

from docopt import docopt

from snapcraft import (
    log,
    commands,
)

_VALID_COMMANDS = [
    'list-parts',
    'list-plugins',
    'init',
    'add-part',
    'pull',
    'build',
    'clean',
    'cleanbuild',
    'stage',
    'strip',
    'snap',
    'help',
    'login',
    'logout',
    'upload',
]

try:
    version = pkg_resources.require('snapcraft')[0].version
except pkg_resources.DistributionNotFound:
    version = 'devel'


def main():
    log.configure()
    args = docopt(__doc__,
                  version=version,
                  options_first=True)
    if args['COMMAND'] not in _VALID_COMMANDS:
        sys.exit('Command {!r} was not recognized'.format(args['COMMAND']))

    try:
        commands.load(args['COMMAND']).main(argv=args['ARGS'])
    except Exception as e:
        sys.exit(textwrap.fill(str(e)))


if __name__ == '__main__':
    main()
