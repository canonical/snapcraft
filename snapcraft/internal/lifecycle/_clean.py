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
from snapcraft.internal import errors, project_loader, mountinfo, steps
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
    max_step = None
    for part in config.all_parts:
        with contextlib.suppress(errors.NoLatestStepError):
            step = part.latest_step()
            if not max_step or step > max_step:
                    max_step = step

    next_step = steps.next_step(max_step)
    if next_step:
        _cleanup_common_directories_for_step(next_step, project_options)


def _cleanup_common_directories_for_step(step, project_options, parts=None):
    if not parts:
        parts = []

    being_tried = False
    if step <= steps.PRIME:
        # Remove the priming area. Only remove the actual 'prime' directory if
        # it's NOT being used in 'snap try'. We'll know that if it's
        # bind-mounted somewhere.
        mounts = mountinfo.MountInfo()
        try:
            mounts.for_root(project_options.prime_dir)
        except errors.RootNotMountedError:
            remove_dir = True
            message = 'Cleaning up priming area'
        else:
            remove_dir = False
            message = ("Cleaning up priming area, but not removing as it's in "
                       "use by 'snap try'")
            being_tried = True
        _cleanup_common(
            project_options.prime_dir, steps.PRIME, message, parts,
            remove_dir=remove_dir)

    if step <= steps.STAGE:
        # Remove the staging area.
        _cleanup_common(
            project_options.stage_dir, steps.STAGE, 'Cleaning up staging area',
            parts)

    if step <= steps.PULL:
        # Remove the parts directory (but leave local plugins alone).
        _cleanup_parts_dir(
            project_options.parts_dir, project_options.local_plugins_dir,
            parts)
        _cleanup_internal_snapcraft_dir()

    if not being_tried:
        _remove_directory_if_empty(project_options.prime_dir)
    _remove_directory_if_empty(project_options.stage_dir)
    _remove_directory_if_empty(project_options.parts_dir)


def _cleanup_common(directory, step, message, parts, *, remove_dir=True):
    if os.path.isdir(directory):
        logger.info(message)
        if remove_dir:
            shutil.rmtree(directory)
        else:
            # Don't delete the parent directory, but delete its contents
            for f in os.scandir(directory):
                if f.is_dir(follow_symlinks=False):
                    shutil.rmtree(f.path)
                elif f.is_file(follow_symlinks=False):
                    os.remove(f.path)
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
        part.mark_cleaned(steps.BUILD)
        part.mark_cleaned(steps.PULL)


def _cleanup_internal_snapcraft_dir():
    if os.path.exists(constants.SNAPCRAFT_INTERNAL_DIR):
        shutil.rmtree(constants.SNAPCRAFT_INTERNAL_DIR)


def clean(project_options, parts, step=None):
    # step defaults to None because that's how it comes from docopt when it's
    # not set.
    if not step:
        step = steps.PULL

    if not parts and step == steps.PULL:
        _cleanup_common_directories_for_step(step, project_options)
        return

    config = project_loader.load_config()

    if not parts and (step == steps.STAGE or step == steps.PRIME):
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

    staged_state = config.get_project_state(steps.STAGE)
    primed_state = config.get_project_state(steps.PRIME)

    _clean_parts(parts, step, config, staged_state, primed_state)

    _cleanup_common_directories(config, project_options)
