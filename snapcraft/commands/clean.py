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
  -s STEP --step=STEP   only clean the specified step and those that depend
                        upon it.
"""

import os
import logging
import shutil

from docopt import docopt

import snapcraft.yaml
from snapcraft import common

logger = logging.getLogger(__name__)


def _reverse_dependency_tree(config, part_name):
    dependents = config.part_dependents(part_name)
    for dependent in dependents.copy():
        # No need to worry about infinite recursion due to circular
        # dependencies since the YAML validation won't allow it.
        dependents |= _reverse_dependency_tree(config, dependent)

    return dependents


def _clean_part_and_all_dependents(config, part, staged_state, stripped_state,
                                   step):
    cleaned_parts = set()

    # Clean the part in question
    part.clean(staged_state, stripped_state, step)
    cleaned_parts.add(part.name)

    # Now obtain the reverse dependency tree for this part. Make sure
    # all dependents are also cleaned.
    dependents = _reverse_dependency_tree(config, part.name)
    cleaned_parts |= dependents
    dependent_parts = {p for p in config.all_parts
                       if p.name in dependents}
    for dependent_part in dependent_parts:
        dependent_part.clean(staged_state, stripped_state, step)

    return cleaned_parts


def main(argv=None):
    argv = argv if argv else []
    args = docopt(__doc__, argv=argv)

    config = snapcraft.yaml.load_config()

    if args['PART']:
        config.validate_parts(args['PART'])

    staged_state = config.get_project_state('stage')
    stripped_state = config.get_project_state('strip')

    cleaned_parts = set()
    for part in config.all_parts:
        if not args['PART']:
            part.clean(staged_state, stripped_state, args['--step'])
            cleaned_parts.add(part.name)
        elif part.name in args['PART']:
            cleaned_parts |= _clean_part_and_all_dependents(
                config, part, staged_state, stripped_state, args['--step'])

    # parts dir does not contain only generated code, so only blow it away if
    # there's nothing left inside it.
    if (os.path.exists(common.get_partsdir()) and
            not os.listdir(common.get_partsdir())):
        os.rmdir(common.get_partsdir())

    parts_match = set(config.part_names) == cleaned_parts
    # Only clean stage if all the parts were cleaned up.
    clean_stage = not args['PART'] or parts_match
    if clean_stage and os.path.exists(common.get_stagedir()):
        logger.info('Cleaning up staging area')
        shutil.rmtree(common.get_stagedir())

    if os.path.exists(common.get_snapdir()):
        logger.info('Cleaning up snapping area')
        shutil.rmtree(common.get_snapdir())
