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
snapcraft clean

Remove content - download, build or install artifacts.

Usage:
  clean [options] [PART ...]

Options:
  -h --help             show this help message and exit.
"""

import os
import logging
import shutil

from docopt import docopt

import snapcraft.yaml
from snapcraft import common

logger = logging.getLogger(__name__)


def main(argv=None):
    argv = argv if argv else []
    args = docopt(__doc__, argv=argv)

    config = snapcraft.yaml.load_config()

    if args['PART']:
        config.validate_parts(args['PART'])

    for part in config.all_parts:
        if not args['PART'] or part.name in args['PART']:
            part.clean()

    # parts dir does not contain only generated code.
    if (os.path.exists(common.get_partsdir()) and
            not os.listdir(common.get_partsdir())):
        os.rmdir(common.get_partsdir())

    parts_match = set(config.part_names) == set(args['PART'])
    # Only clean stage if all the parts were cleaned up.
    clean_stage = not args['PART'] or parts_match
    if clean_stage and os.path.exists(common.get_stagedir()):
        logger.info('Cleaning up staging area')
        shutil.rmtree(common.get_stagedir())

    if os.path.exists(common.get_snapdir()):
        logger.info('Cleaning up snapping area')
        shutil.rmtree(common.get_snapdir())
