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
  upload [options] SNAP_FILE

Options:
  -h --help             show this help message and exit.

"""

import logging
import os
import subprocess
import tempfile

import yaml
from docopt import docopt

from snapcraft.config import load_config
from snapcraft.storeapi import upload


logger = logging.getLogger(__name__)


def _get_name_from_snap_file(snap_path):
    with tempfile.TemporaryDirectory() as temp_dir:
        subprocess.check_call(
            ['unsquashfs', '-d', os.path.join(temp_dir, 'squashfs-root'),
             snap_path, '-e', os.path.join('meta', 'snap.yaml')])
        with open(os.path.join(
                temp_dir, 'squashfs-root', 'meta', 'snap.yaml')) as yaml_file:
            snap_yaml = yaml.load(yaml_file)

    return snap_yaml['name']


def main(argv=None, project_options=None):
    """Upload snap package to the Ubuntu Store."""
    argv = argv if argv else []
    args = docopt(__doc__, argv=argv)

    snap_filename = args['SNAP_FILE']
    if not os.path.exists(snap_filename):
        raise FileNotFoundError(snap_filename)
    else:
        snap_name = _get_name_from_snap_file(snap_filename)
        logger.info('Uploading existing {}.'.format(snap_filename))

        config = load_config(project_options)
        upload(snap_filename, snap_name, config=config)
