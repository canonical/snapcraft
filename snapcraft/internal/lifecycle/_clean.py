# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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


logger = logging.getLogger(__name__)


def _clean_part(part_name, step, config, staged_state, primed_state):
    if step.next_step() is None:
        template = "Cleaning {step} step for {part}"
    else:
        template = "Cleaning {step} step (and all subsequent steps) for {part}"

    logger.info(template.format(step=step.name, part=part_name))
    config.parts.clean_part(part_name, staged_state, primed_state, step)

    # If we just cleaned the root of a dependency tree. Return all dirty
    # dependents.
    return get_dirty_reverse_dependencies(part_name, step, config)


def get_dirty_reverse_dependencies(part_name, step, config):
    # If other parts are depending on this step of the part, they're now
    # dirty
    dirty_reverse_dependencies = set()
    reverse_dependencies = config.parts.get_reverse_dependencies(
        part_name, recursive=True
    )
    dirty_step = steps.dirty_step_if_dependency_changes(step)
    for reverse_dependency in reverse_dependencies:
        if not reverse_dependency.should_step_run(dirty_step):
            dirty_reverse_dependencies.add(reverse_dependency)

    return dirty_reverse_dependencies


def _clean_parts(part_names, step, config, staged_state, primed_state):
    if not step:
        step = steps.next_step(None)

    for part_name in part_names:
        dirty_parts = _clean_part(part_name, step, config, staged_state, primed_state)
        dirty_part_names = {p.name for p in dirty_parts}

        parts_not_being_cleaned = dirty_part_names.difference(part_names)
        if parts_not_being_cleaned:
            logger.warning(
                "Cleaned {!r}, which makes the following {} out of date: "
                "{}".format(
                    part_name,
                    formatting_utils.pluralize(
                        parts_not_being_cleaned, "part", "parts"
                    ),
                    formatting_utils.humanize_list(parts_not_being_cleaned, "and"),
                )
            )


def _remove_directory(directory: str) -> None:
    if os.path.isdir(directory):
        shutil.rmtree(directory)


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
            message = "Cleaning up priming area"
        else:
            remove_dir = False
            message = (
                "Cleaning up priming area, but not removing as it's in "
                "use by 'snap try'"
            )
            being_tried = True
        _cleanup_common(
            project_options.prime_dir,
            steps.PRIME,
            message,
            parts,
            remove_dir=remove_dir,
        )

    if step <= steps.STAGE:
        # Remove the staging area.
        _cleanup_common(
            project_options.stage_dir, steps.STAGE, "Cleaning up staging area", parts
        )

    if step <= steps.PULL:
        # Remove the parts directory (but leave local plugins alone).
        _cleanup_parts_dir(
            project_options.parts_dir, project_options.local_plugins_dir, parts
        )
        _remove_directory(project_options._internal_dir)

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
        logger.info("Cleaning up parts directory")
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


def clean(project_options, parts, step=None):
    # step defaults to None because that's how it comes from docopt when it's
    # not set.
    if not step:
        step = steps.PULL

    if not parts and step == steps.PULL:
        _cleanup_common_directories_for_step(step, project_options)
        return

    config = project_loader.load_config(project_options)

    if not parts and step <= steps.PRIME:
        # If we've been asked to clean stage or prime without being given
        # specific parts, just blow away those directories instead of
        # doing it per part (it would just be a waste of time).
        _cleanup_common_directories_for_step(
            step, project_options, parts=config.all_parts
        )

        # No need to continue if that's all that was required
        if step >= steps.STAGE:
            return

    if parts:
        config.parts.validate(parts)
    else:
        parts = [part.name for part in config.all_parts]

    staged_state = config.get_project_state(steps.STAGE)
    primed_state = config.get_project_state(steps.PRIME)

    _clean_parts(parts, step, config, staged_state, primed_state)

    _cleanup_common_directories(config, project_options)
