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
snapcraft help

Initialize a snapcraft project.

Usage:
  init [options]

Options:
  -h --help             show this help message and exit.

"""

import logging
import os.path
import sys

from docopt import docopt

logger = logging.getLogger(__name__)

_TEMPLATE_YAML = r'''name: # the name of the snap
version: # the version of the snap
summary: # 79 char long summary
description: # A longer description for the snap
icon: # A path to an icon for the package - this is optional.
'''


def main(argv=None):
    argv = argv if argv else []
    docopt(__doc__, argv=argv)

    if os.path.exists('snapcraft.yaml'):
        logger.error('snapcraft.yaml already exists!')
        sys.exit(1)
    yaml = _TEMPLATE_YAML.strip()
    with open('snapcraft.yaml', mode='w+') as f:
        f.write(yaml)
    logger.info('Created snapcraft.yaml.')
