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
                        upon it. STEP can be one of: pull, build, stage, strip,
                        snap. See 'snapcraft help plugins' to learn more.
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
    # Clean the part in question
    part.clean(staged_state, stripped_state, step)

    # Now obtain the reverse dependency tree for this part. Make sure
    # all dependents are also cleaned.
    dependents = _reverse_dependency_tree(config, part.name)
    dependent_parts = {p for p in config.all_parts
                       if p.name in dependents}
    for dependent_part in dependent_parts:
        dependent_part.clean(staged_state, stripped_state, step)


def _remove_directory_if_empty(directory):
    if os.path.isdir(directory) and not os.listdir(directory):
        os.rmdir(directory)


def _cleanup_common_directories(config):
    _remove_directory_if_empty(common.get_partsdir())
    _remove_directory_if_empty(common.get_stagedir())
    _remove_directory_if_empty(common.get_snapdir())

    max_index = -1
    for part in config.all_parts:
        step = part.last_step()
        if step:
            index = common.COMMAND_ORDER.index(step)
            if index > max_index:
                max_index = index

    # If no parts have been pulled, remove the parts directory. In most cases
    # this directory should have already been cleaned, but this handles the
    # case of a failed pull.
    should_remove_partsdir = max_index < common.COMMAND_ORDER.index('pull')
    if should_remove_partsdir and os.path.exists(common.get_partsdir()):
        logger.info('Cleaning up parts directory')
        shutil.rmtree(common.get_partsdir())

    # If no parts have been staged, remove staging area.
    should_remove_stagedir = max_index < common.COMMAND_ORDER.index('stage')
    if should_remove_stagedir and os.path.exists(common.get_stagedir()):
        logger.info('Cleaning up staging area')
        shutil.rmtree(common.get_stagedir())

    # If no parts have been stripped, remove snapping area.
    should_remove_snapdir = max_index < common.COMMAND_ORDER.index('strip')
    if should_remove_snapdir and os.path.exists(common.get_snapdir()):
        logger.info('Cleaning up snapping area')
        shutil.rmtree(common.get_snapdir())


def main(argv=None):
    argv = argv if argv else []
    args = docopt(__doc__, argv=argv)

    config = snapcraft.yaml.load_config()

    if args['PART']:
        config.validate_parts(args['PART'])

    staged_state = config.get_project_state('stage')
    stripped_state = config.get_project_state('strip')

    for part in config.all_parts:
        if not args['PART']:
            part.clean(staged_state, stripped_state, args['--step'])
        elif part.name in args['PART']:
            _clean_part_and_all_dependents(
                config, part, staged_state, stripped_state, args['--step'])

    _cleanup_common_directories(config)
