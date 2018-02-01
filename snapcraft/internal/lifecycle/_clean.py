# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
import contextlib
import logging
import os
import shutil

from snapcraft import formatting_utils
from snapcraft.internal import common, project_loader
from . import constants


logger = logging.getLogger(__name__)


def _reverse_dependency_tree(config, part_name):
    dependents = config.parts.get_dependents(part_name)
    for dependent in dependents.copy():
        # No need to worry about infinite recursion due to circular
        # dependencies since the YAML validation won't allow it.
        dependents |= _reverse_dependency_tree(config, dependent)

    return dependents


def _clean_part_and_all_dependents(part_name, step, config, staged_state,
                                   primed_state):
    # Obtain the reverse dependency tree for this part. Make sure all
    # dependents are cleaned.
    dependents = _reverse_dependency_tree(config, part_name)
    dependent_parts = {p for p in config.all_parts
                       if p.name in dependents}
    for dependent_part in dependent_parts:
        dependent_part.clean(staged_state, primed_state, step)

    # Finally, clean the part in question
    config.parts.clean_part(part_name, staged_state, primed_state, step)


def _verify_dependents_will_be_cleaned(part_name, clean_part_names, step,
                                       config):
    # Get the name of the parts that depend upon this one
    dependents = config.parts.get_dependents(part_name)
    additional_dependents = []

    # Verify that they're either already clean, or that they will be cleaned.
    if not dependents.issubset(clean_part_names):
        for part in config.all_parts:
            if part.name in dependents and not part.is_clean(step):
                humanized_parts = formatting_utils.humanize_list(
                    dependents, 'and')
                additional_dependents.append(part_name)

                logger.warning(
                    'Requested clean of {!r} which requires also cleaning '
                    'the part{} {}'.format(part_name,
                                           '' if len(dependents) == 1 else 's',
                                           humanized_parts))


def _clean_parts(part_names, step, config, staged_state, primed_state):
    if not step:
        step = 'pull'

    # Before doing anything, verify that we weren't asked to clean only the
    # root of a dependency tree and hint that more parts would be cleaned
    # if not.
    for part_name in part_names:
        _verify_dependents_will_be_cleaned(part_name, part_names, step, config)

    # Now we can actually clean.
    for part_name in part_names:
        _clean_part_and_all_dependents(
            part_name, step, config, staged_state, primed_state)


def _remove_directory_if_empty(directory):
    if os.path.isdir(directory) and not os.listdir(directory):
        os.rmdir(directory)


def _cleanup_common_directories(config, project_options):
    max_index = -1
    for part in config.all_parts:
        step = part.last_step()
        if step:
            index = common.COMMAND_ORDER.index(step)
            if index > max_index:
                max_index = index

    with contextlib.suppress(IndexError):
        _cleanup_common_directories_for_step(
            common.COMMAND_ORDER[max_index+1], project_options)


def _cleanup_common_directories_for_step(step, project_options, parts=None):
    if not parts:
        parts = []

    index = common.COMMAND_ORDER.index(step)

    if index <= common.COMMAND_ORDER.index('prime'):
        # Remove the priming area.
        _cleanup_common(
            project_options.prime_dir, 'prime', 'Cleaning up priming area',
            parts)

    if index <= common.COMMAND_ORDER.index('stage'):
        # Remove the staging area.
        _cleanup_common(
            project_options.stage_dir, 'stage', 'Cleaning up staging area',
            parts)

    if index <= common.COMMAND_ORDER.index('pull'):
        # Remove the parts directory (but leave local plugins alone).
        _cleanup_parts_dir(
            project_options.parts_dir, project_options.local_plugins_dir,
            parts)
        _cleanup_internal_snapcraft_dir()

    _remove_directory_if_empty(project_options.prime_dir)
    _remove_directory_if_empty(project_options.stage_dir)
    _remove_directory_if_empty(project_options.parts_dir)


def _cleanup_common(directory, step, message, parts):
    if os.path.isdir(directory):
        logger.info(message)
        shutil.rmtree(directory)
    for part in parts:
        part.mark_cleaned(step)


def _cleanup_parts_dir(parts_dir, local_plugins_dir, parts):
    if os.path.exists(parts_dir):
        logger.info('Cleaning up parts directory')
        for subdirectory in os.listdir(parts_dir):
            path = os.path.join(parts_dir, subdirectory)
            if path != local_plugins_dir:
                try:
                    shutil.rmtree(path)
                except NotADirectoryError:
                    os.remove(path)
    for part in parts:
        part.mark_cleaned('build')
        part.mark_cleaned('pull')


def _cleanup_internal_snapcraft_dir():
    if os.path.exists(constants.SNAPCRAFT_INTERNAL_DIR):
        shutil.rmtree(constants.SNAPCRAFT_INTERNAL_DIR)


def clean(project_options, parts, step=None):
    # step defaults to None because that's how it comes from docopt when it's
    # not set.
    if not step:
        step = 'pull'

    if not parts and step == 'pull':
        _cleanup_common_directories_for_step(step, project_options)
        return

    config = project_loader.load_config()

    if not parts and (step == 'stage' or step == 'prime'):
        # If we've been asked to clean stage or prime without being given
        # specific parts, just blow away those directories instead of
        # doing it per part.
        _cleanup_common_directories_for_step(
            step, project_options, parts=config.all_parts)
        return

    if parts:
        config.parts.validate(parts)
    else:
        parts = [part.name for part in config.all_parts]

    staged_state = config.get_project_state('stage')
    primed_state = config.get_project_state('prime')

    _clean_parts(parts, step, config, staged_state, primed_state)

    _cleanup_common_directories(config, project_options)
