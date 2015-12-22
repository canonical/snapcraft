# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
snapcraft upload

Upload snap package to the Ubuntu Store.

Usage:
  upload [options]

Options:
  -h --help             show this help message and exit.

"""

import logging
import os

from docopt import docopt

import snapcraft.yaml
from snapcraft.commands import snap
from snapcraft.config import load_config
from snapcraft.storeapi import upload


logger = logging.getLogger(__name__)


def _format_snap_name(snap):
    snap['arch'] = (snap['architectures'][0]
                    if len(snap['architectures']) == 1 else 'multi')
    return '{name}_{version}_{arch}.snap'.format(**snap)


def main(argv=None):
    """Upload snap package to the Ubuntu Store."""
    argv = argv if argv else []
    docopt(__doc__, argv=argv)

    # make sure the full lifecycle is executed
    yaml_config = snapcraft.yaml.load_config()
    snap_name = _format_snap_name(yaml_config.data)

    if not os.path.exists(snap_name):
        logger.info(
            'Snap {} not found. Running snap step to create it.'.format(
                snap_name))
        snap.main(argv=argv)

    logger.info('Uploading {}'.format(snap_name))
    config = load_config()
    success = upload(snap_name, config=config)
    if success:
        logger.info('{} upload complete'.format(snap_name))
    else:
        logger.info('{} upload failed'.format(snap_name))
