# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018,2020 Canonical Ltd
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

import collections
import contextlib
from typing import Any, Dict, List, Optional, Set

import snapcraft.internal.project_loader._config as _config
from snapcraft.internal import errors, pluginhandler, steps

_DirtyReport = Dict[str, Dict[steps.Step, Optional[pluginhandler.DirtyReport]]]
_OutdatedReport = Dict[str, Dict[steps.Step, Optional[pluginhandler.OutdatedReport]]]


class StatusCache:
    """The StatusCache is a lazy caching interface for the status of parts."""

    def __init__(self, config: _config.Config) -> None:
        """Create a new StatusCache.

        :param _config.Config config: Project config.
        """
        self.config = config
        self._steps_run: Dict[str, Set[steps.Step]] = dict()
        self._outdated_reports: _OutdatedReport = collections.defaultdict(dict)
        self._dirty_reports: _DirtyReport = collections.defaultdict(dict)

    def should_step_run(
        self, part: pluginhandler.PluginHandler, step: steps.Step
    ) -> bool:
        """Determine if a given step of a given part should run.

        :param pluginhandler.PluginHandler part: Part in question.
        :param steps.Step step: Step in question.
        :return: Whether or not step should run.
        :rtype: bool

        A given step should run if it:
            1. Hasn't yet run
            2. Is dirty
            3. Is outdated
            4. Either (1), (2), or (3) apply to any earlier steps in the part's
               lifecycle
        """
        if (
            not self.has_step_run(part, step)
            or self.get_outdated_report(part, step) is not None
            or self.get_dirty_report(part, step) is not None
        ):
            return True

        previous_step = step.previous_step()
        if previous_step:
            return self.should_step_run(part, previous_step)

        return False

    def add_step_run(self, part: pluginhandler.PluginHandler, step: steps.Step) -> None:
        """Cache the fact that a given step has now run for the given part.

        :param pluginhandler.PluginHandler part: Part in question.
        :param steps.Step step: Step in question.
        """
        self._ensure_steps_run(part)
        self._steps_run[part.name].add(step)

    def has_step_run(self, part: pluginhandler.PluginHandler, step: steps.Step) -> bool:
        """Determine if a given step of a given part has already run.

        :param pluginhandler.PluginHandler part: Part in question.
        :param steps.Step step: Step in question.
        :return: Whether or not the step has run.
        :rtype: bool
        """
        self._ensure_steps_run(part)
        return step in self._steps_run[part.name]

    def get_outdated_report(self, part, step):
        """Obtain the outdated report for a given step of the given part.

        :param pluginhandler.PluginHandler part: Part in question.
        :param steps.Step step: Step in question.
        :return: Outdated report (could be None)
        :rtype: pluginhandler.OutdatedReport
        """
        self._ensure_outdated_report(part, step)
        return self._outdated_reports[part.name][step]

    def get_dirty_report(
        self, part: pluginhandler.PluginHandler, step: steps.Step
    ) -> Optional[pluginhandler.DirtyReport]:
        """Obtain the dirty report for a given step of the given part.

        :param pluginhandler.PluginHandler part: Part in question.
        :param steps.Step step: Step in question.
        :return: Dirty report (could be None)
        :rtype: pluginhandler.DirtyReport
        """
        self._ensure_dirty_report(part, step)
        return self._dirty_reports[part.name][step]

    def clear_step(self, part: pluginhandler.PluginHandler, step: steps.Step) -> None:
        """Clear the given step of the given part from the cache.

        :param pluginhandler.PluginHandler part: Part in question.
        :param steps.Step step: Step in question.

        This function does nothing if the step wasn't cached.
        """
        if part.name in self._steps_run:
            _remove_key(self._steps_run[part.name], step)
            if not self._steps_run[part.name]:
                _del_key(self._steps_run, part.name)
        _del_key(self._outdated_reports[part.name], step)
        if not self._outdated_reports[part.name]:
            _del_key(self._outdated_reports, part.name)
        _del_key(self._dirty_reports[part.name], step)
        if not self._dirty_reports[part.name]:
            _del_key(self._dirty_reports, part.name)

    def _ensure_steps_run(self, part: pluginhandler.PluginHandler) -> None:
        if part.name not in self._steps_run:
            self._steps_run[part.name] = _get_steps_run(part)

    def _ensure_outdated_report(
        self, part: pluginhandler.PluginHandler, step: steps.Step
    ) -> None:
        if step not in self._outdated_reports[part.name]:
            self._outdated_reports[part.name][step] = part.get_outdated_report(step)

    def _ensure_dirty_report(
        self, part: pluginhandler.PluginHandler, step: steps.Step
    ) -> None:
        # If we already have a dirty report, bail
        if step in self._dirty_reports[part.name]:
            return

        # Get the dirty report from the PluginHandler. If it's dirty, we can
        # stop here
        self._dirty_reports[part.name][step] = part.get_dirty_report(step)

        # The dirty report from the PluginHandler only takes into account
        # properties specific to that part. If it's not dirty because of those,
        # we need to expand it here to also take its dependencies (if any) into
        # account
        prerequisite_step = steps.get_dependency_prerequisite_step(step)
        dependencies = self.config.parts.get_dependencies(part.name, recursive=True)

        changed_dependencies: List[pluginhandler.Dependency] = []
        with contextlib.suppress(errors.StepHasNotRunError):
            timestamp = part.step_timestamp(step)
            for dependency in dependencies:
                # Make sure the prerequisite step of this dependency has not
                # run more recently than (or should run _before_) this step.
                try:
                    prerequisite_timestamp = dependency.step_timestamp(
                        prerequisite_step
                    )
                except errors.StepHasNotRunError:
                    dependency_changed = True
                else:
                    dependency_changed = timestamp < prerequisite_timestamp

                if dependency_changed or self.should_step_run(
                    dependency, prerequisite_step
                ):
                    changed_dependencies.append(
                        pluginhandler.Dependency(
                            part_name=dependency.name, step=prerequisite_step
                        )
                    )

            if changed_dependencies:
                self._dirty_reports[part.name][step] = pluginhandler.DirtyReport(
                    changed_dependencies=changed_dependencies
                )


def _get_steps_run(part: pluginhandler.PluginHandler) -> Set[steps.Step]:
    steps_run = set()  # type: Set[steps.Step]
    for step in steps.STEPS:
        if not part.should_step_run(step):
            steps_run.add(step)

    return steps_run


def _del_key(c: Dict[Any, Any], key: Any) -> None:
    with contextlib.suppress(KeyError):
        del c[key]


def _remove_key(c: Set[Any], key: Any) -> None:
    with contextlib.suppress(KeyError):
        c.remove(key)
