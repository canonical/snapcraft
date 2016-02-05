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
snapcraft cleanbuild

Creates a lxd container to build the snap.

This is a way to guarantee that the snapcraft.yaml used with associated
local sources is not using any dependencies local to the developer system.

The cleanbuild command requires a properly setup lxd environment that
can connect to external networks. Refer to the "Ubuntu Desktop and
Ubuntu Server" section on
https://linuxcontainers.org/lxd/getting-started-cli
to get started.

Usage:
  cleanbuild [options]

Options:
  -h --help             show this help message and exit.
"""

import logging
import os.path
import tarfile

from docopt import docopt

from snapcraft import repo
from snapcraft.lxd import Cleanbuilder
from snapcraft.common import format_snap_name

from snapcraft.yaml import load_config

logger = logging.getLogger(__name__)


def _create_tar_filter(tar_filename):
    def _tar_filter(tarinfo):
        fn = tarinfo.name
        if fn.startswith('./parts/') and not fn.startswith('./parts/plugins'):
            return None
        elif fn in ('./stage', './snap', tar_filename):
            return None
        elif fn.endswith('.snap'):
            return None
        return tarinfo
    return _tar_filter


def main(argv=None):
    argv = [] if argv is None else argv
    docopt(__doc__, argv=argv)

    if not repo.is_package_installed('lxd'):
        raise EnvironmentError(
            'The lxd package is not installed, in order to use `cleanbuild` '
            'you must install lxd onto your system. Refer to the '
            '"Ubuntu Desktop and Ubuntu Server" section on '
            'https://linuxcontainers.org/lxd/getting-started-cli/'
            '#ubuntu-desktop-and-ubuntu-server to enable a proper setup.')

    config = load_config()
    tar_filename = '{}_{}_source.tar.bz2'.format(
        config.data['name'], config.data['version'])

    with tarfile.open(tar_filename, 'w:bz2') as t:
        t.add(os.path.curdir, filter=_create_tar_filter(tar_filename))

    snap_filename = format_snap_name(config.data)
    Cleanbuilder(snap_filename, tar_filename).execute()
