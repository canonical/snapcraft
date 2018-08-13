# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import os
from typing import Callable, Iterable, Set, Tuple, Union  # noqa: F401

from snapcraft import project
from snapcraft.internal import pluginhandler, states, steps
from . import errors


def provides(
    path: str, project: project.Project, parts: Iterable[pluginhandler.PluginHandler]
) -> Set[pluginhandler.PluginHandler]:
    """Determine which part(s) provide(s) the given path.

    :param str path: Path to the directory or file in question.
    :param project.Project project: Project settings.
    :param iterable parts: Iterable of all parts.
    :returns: Set of parts that provide the given path.
    """
    # First of all, ensure the file actually exists before doing any work
    if not os.path.exists(path):
        raise errors.NoSuchFileError(path)

    # Convert file path into absolute path
    absolute_file_path = os.path.abspath(path)

    # Which step are we operating on? We'll know by where the file_path is:
    # the staging area, or the priming area?
    if absolute_file_path.startswith(project.stage_dir):
        step = steps.STAGE
        relative_file_path = os.path.relpath(
            absolute_file_path, start=project.stage_dir
        )
    elif absolute_file_path.startswith(project.prime_dir):
        step = steps.PRIME
        relative_file_path = os.path.relpath(
            absolute_file_path, start=project.prime_dir
        )
    else:
        raise errors.ProvidesInvalidFilePathError(path)

    is_dir = os.path.isdir(absolute_file_path)
    is_file = os.path.isfile(absolute_file_path)

    providing_parts = set()  # type: Set[pluginhandler.PluginHandler]
    for part, state in _part_states_for_step(step, parts):
        if is_dir and relative_file_path in state.directories:
            providing_parts.add(part)
        elif is_file and relative_file_path in state.files:
            providing_parts.add(part)

    if not providing_parts:
        raise errors.UntrackedFileError(path)

    return providing_parts


# Implemented as a generator since loading up the state could be heavy
def _part_states_for_step(
    step: steps.Step, parts: Iterable[pluginhandler.PluginHandler]
) -> Iterable[
    Tuple[pluginhandler.PluginHandler, Union[states.StageState, states.PrimeState]]
]:
    state_getter = (
        None
    )  # type: Callable[[pluginhandler.PluginHandler], Union[states.StageState, states.PrimeState]]

    if step == steps.STAGE:
        state_getter = pluginhandler.PluginHandler.get_stage_state
    elif step == steps.PRIME:
        state_getter = pluginhandler.PluginHandler.get_prime_state
    else:
        # This is programmer error
        raise RuntimeError("Only the stage or prime step is supported!")

    for part in parts:
        state = state_getter(part)
        if state:
            yield (part, state)
