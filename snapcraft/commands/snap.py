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
snapcraft snap

Creates a snap.

Usage:
  snap [options] [DIRECTORY]

Options:
  DIRECTORY               optional target directory to snap.
  -h --help               show this help message and exit.
  -o SNAP --output SNAP   create snap with a specific filename

"""

import logging
import os.path
import subprocess

import yaml
from docopt import docopt

from snapcraft import (
    common,
    lifecycle,
)


logger = logging.getLogger(__name__)


def _snap_data_from_dir(dir):
    with open(os.path.join(dir, 'meta', 'snap.yaml')) as f:
        snap = yaml.load(f)

    return {'name': snap['name'],
            'version': snap['version'],
            'arch': snap['architectures'],
            'type': snap.get('type', '')}


def _format_snap_name(snap):
    snap['arch'] = snap['arch'][0] if len(snap['arch']) == 1 else 'multi'
    return '{name}_{version}_{arch}.snap'.format(**snap)


def main(argv=None):
    argv = argv if argv else []
    args = docopt(__doc__, argv=argv)

    if args['DIRECTORY']:
        snap_dir = os.path.abspath(args['DIRECTORY'])
        snap = _snap_data_from_dir(snap_dir)
    else:
        # make sure the full lifecycle is executed
        snap_dir = common.get_snapdir()
        snap = lifecycle.execute('strip')

    snap_name = args['--output'] or _format_snap_name(snap)

    logger.info('Snapping {}'.format(snap_name))
    # These options need to match the review tools:
    # http://bazaar.launchpad.net/~click-reviewers/click-reviewers-tools/trunk/view/head:/clickreviews/common.py#L38
    mksquashfs_args = ['-noappend', '-comp', 'xz', '-no-xattrs']
    if snap['type'] != 'os':
        mksquashfs_args.append('-all-root')

    subprocess.check_call(
        ['mksquashfs', snap_dir, snap_name] + mksquashfs_args)
    logger.info('Snapped {}'.format(snap_name))
