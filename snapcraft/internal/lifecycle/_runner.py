# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

import logging
from typing import List, Optional, Sequence, Set

from snapcraft import config, plugins, storeapi
from snapcraft.internal import (
    common,
    errors,
    pluginhandler,
    project_loader,
    repo,
    states,
    steps,
)
from snapcraft.internal.meta._snap_packaging import create_snap_packaging
from snapcraft.internal.pluginhandler._part_environment import (
    get_snapcraft_part_directory_environment,
)

from ._status_cache import StatusCache

logger = logging.getLogger(__name__)


def _get_required_grade(*, base: Optional[str], arch: str) -> str:
    # Some types of snap do not require a base.
    if base is None:
        return "stable"

    # This will make a request over the network.
    # We use storeapi instead of repo.snaps so this can work under Docker
    # and related environments.
    try:
        base_info = storeapi.StoreClient().snap.get_info(base)
        base_info.get_channel_mapping(risk="stable", arch=arch)
    except storeapi.errors.SnapNotFoundError:
        return "devel"
    else:
        return "stable"


def _install_build_packages(build_packages: Set[str]) -> List[str]:
    return repo.Repo.install_build_packages(build_packages)


def _install_build_snaps(build_snaps: Set[str], content_snaps: Set[str]) -> List[str]:
    if common.is_process_container() and build_snaps:
        installed_snaps: List[str] = []
        logger.warning(
            (
                "The following snaps are required but not installed as snapcraft "
                "is running inside docker or podman container: {}.\n"
                "Please ensure the environment is properly setup before continuing.\n"
                "Ignore this message if the appropriate measures have already been taken".format(
                    ", ".join(build_snaps)
                )
            )
        )
    else:
        installed_snaps = repo.snaps.install_snaps(build_snaps)
        for content_snap in content_snaps:
            try:
                installed_snaps += repo.snaps.install_snaps([content_snap])
            except repo.snaps.errors.SnapUnavailableError:
                logger.warning(
                    f"Could not install snap defined in plug {content_snap!r}. "
                    "The missing library report may have false positives listed if those "
                    "libraries are provided by the content snap."
                )

    return installed_snaps


def execute(
    step: steps.Step,
    project_config: "project_loader._config.Config",
    part_names: Sequence[str] = None,
):
    """Execute until step in the lifecycle for part_names or all parts.

    Lifecycle execution will happen for each step iterating over all
    the available parts, if part_names is specified, only those parts
    will run.

    If one of the parts to execute has an after keyword, execution is
    forced until the stage step for such part. If part_names was provided
    and after is not in this set, an exception will be raised.

    :param str step: A valid step in the lifecycle: pull, build, prime or snap.
    :param project_config: Fully loaded project (old logic moving either to
                           Project or the PluginHandler).
    :param list part_names: A list of parts to execute the lifecycle on.
    :raises RuntimeError: If a prerequesite of the part needs to be staged
                          and such part is not in the list of parts to iterate
                          over.
    :returns: A dict with the snap name, version, type and architectures.
    """
    installed_packages = _install_build_packages(project_config.get_build_packages())
    installed_snaps = _install_build_snaps(
        project_config.get_build_snaps(), project_config.project._get_content_snaps()
    )

    try:
        global_state = states.GlobalState.load(
            filepath=project_config.project._get_global_state_file_path()
        )
    except FileNotFoundError:
        global_state = states.GlobalState()
    global_state.append_build_packages(installed_packages)
    global_state.append_build_snaps(installed_snaps)
    # Let's not call out to the Snap Store if we do not need to.
    if global_state.get_required_grade() is None:
        global_state.set_required_grade(
            _get_required_grade(
                base=project_config.project.info.base,
                arch=project_config.project.deb_arch,
            )
        )
    global_state.save(filepath=project_config.project._get_global_state_file_path())

    executor = _Executor(project_config)
    executor.run(step, part_names)
    if not executor.steps_were_run:
        logger.warning(
            "The requested action has already been taken. Consider\n"
            "specifying parts, or clean the steps you want to run again."
        )

    return {
        "name": project_config.data["name"],
        "version": project_config.data.get("version"),
        "arch": project_config.data["architectures"],
        "type": project_config.data.get("type", ""),
    }


def _replace_in_part(part):
    for key, value in part.plugin.options.__dict__.items():
        replacements = project_loader.environment_to_replacements(
            get_snapcraft_part_directory_environment(part)
        )

        value = project_loader.replace_attr(value, replacements)
        setattr(part.plugin.options, key, value)

    return part


class _Executor:
    def __init__(self, project_config):
        self.config = project_config
        self.project = project_config.project
        self.parts_config = project_config.parts
        self.steps_were_run = False

        self._cache = StatusCache(project_config)

    def run(self, step: steps.Step, part_names=None):
        if part_names:
            self.parts_config.validate(part_names)
            # self.config.all_parts is already ordered, let's not lose that
            # and keep using a list.
            parts = [p for p in self.config.all_parts if p.name in part_names]
            processed_part_names = part_names
        else:
            parts = self.config.all_parts
            processed_part_names = self.config.part_names

        with config.CLIConfig() as cli_config:
            for current_step in step.previous_steps() + [step]:
                if current_step == steps.STAGE:
                    # XXX check only for collisions on the parts that have
                    # already been built --elopio - 20170713
                    pluginhandler.check_for_collisions(self.config.all_parts)
                for part in parts:
                    self._handle_step(part_names, part, step, current_step, cli_config)

        self._create_meta(step, processed_part_names)

    def _handle_step(
        self,
        requested_part_names: Sequence[str],
        part: pluginhandler.PluginHandler,
        requested_step: steps.Step,
        current_step: steps.Step,
        cli_config,
    ) -> None:
        # If this step hasn't yet run, all we need to do is run it
        if not self._cache.has_step_run(part, current_step):
            getattr(self, "_run_{}".format(current_step.name))(part)
            return

        # Alright, this step has already run. In that case, a few different
        # things need to happen:

        # 1. By default, if a step has already run, don't run it again.
        #    However, automatically clean and re-run the step if all the
        #    following conditions apply:
        #
        #      1.1. The step is the exact step that was requested (not an
        #           earlier one).
        #      1.2. The part was explicitly specified.
        if (
            requested_part_names
            and current_step == requested_step
            and part.name in requested_part_names
        ):
            getattr(self, "_re{}".format(current_step.name))(part)
            return

        # 2. If a step has already run, it might be dirty, in which case we
        #    need to clean and run it again.
        dirty_report = self._cache.get_dirty_report(part, current_step)
        if dirty_report:
            self._handle_dirty(part, current_step, dirty_report, cli_config)
            return

        # 3. If a step has already run, it might be outdated, in which case we
        #    need to update it (without cleaning if possible).
        outdated_report = self._cache.get_outdated_report(part, current_step)
        if outdated_report:
            self._handle_outdated(part, current_step, outdated_report, cli_config)
            return

        # 4. The step has already run, and is up-to-date, no need to run it
        #    again.
        notify_part_progress(
            part, "Skipping {}".format(current_step.name), "(already ran)"
        )

    def _run_pull(self, part):
        self._run_step(step=steps.PULL, part=part, progress="Pulling")

    def _repull(self, part, hint=""):
        self._rerun_step(
            step=steps.PULL,
            part=part,
            progress="Cleaning later steps and re-pulling",
            hint=hint,
        )

    def _run_build(self, part):
        self._run_step(step=steps.BUILD, part=part, progress="Building")

    def _rebuild(self, part, hint=""):
        self._rerun_step(
            step=steps.BUILD,
            part=part,
            progress="Cleaning later steps and re-building",
            hint=hint,
        )

    def _run_stage(self, part):
        self._run_step(step=steps.STAGE, part=part, progress="Staging")

    def _restage(self, part, hint=""):
        self._rerun_step(
            step=steps.STAGE,
            part=part,
            progress="Cleaning later steps and re-staging",
            hint=hint,
        )

    def _run_prime(self, part):
        self._run_step(step=steps.PRIME, part=part, progress="Priming")

    def _reprime(self, part, hint=""):
        self._rerun_step(step=steps.PRIME, part=part, progress="Re-priming", hint=hint)

    def _prepare_step(self, *, step: steps.Step, part: pluginhandler.PluginHandler):
        common.reset_env()
        all_dependencies = self.parts_config.get_dependencies(part.name)

        # Filter dependencies down to only those that need to run the
        # prerequisite step
        prerequisite_step = steps.get_dependency_prerequisite_step(step)
        dependencies = {
            p
            for p in all_dependencies
            if self._cache.should_step_run(p, prerequisite_step)
        }

        if dependencies:
            dependency_names = {p.name for p in dependencies}
            # Dependencies need to go all the way to the prerequisite step to
            # be able to share the common assets that make them a dependency
            logger.info(
                "{!r} has dependencies that need to be {}d: {}".format(
                    part.name, prerequisite_step.name, " ".join(dependency_names)
                )
            )
            self.run(prerequisite_step, dependency_names)

        # Run the preparation function for this step (if implemented)
        preparation_function = getattr(part, "prepare_{}".format(step.name), None)
        if preparation_function:
            notify_part_progress(part, "Preparing to {}".format(step.name), debug=True)
            preparation_function()

        if isinstance(part.plugin, plugins.v1.PluginV1):
            common.env = self.parts_config.build_env_for_part(part)
            common.env.extend(self.config.project_env())

        part = _replace_in_part(part)

    def _run_step(self, *, step: steps.Step, part, progress, hint=""):
        self._prepare_step(step=step, part=part)

        notify_part_progress(part, progress, hint)
        getattr(part, step.name)()

        # We know we just ran this step, so rather than check, manually twiddle
        # the cache
        self._complete_step(part, step)

    def _complete_step(self, part, step):
        self._cache.clear_step(part, step)
        self._cache.add_step_run(part, step)
        self.steps_were_run = True

    def _rerun_step(self, *, step: steps.Step, part, progress, hint=""):
        staged_state = self.config.get_project_state(steps.STAGE)
        primed_state = self.config.get_project_state(steps.PRIME)

        # First clean the step, then run it again
        part.clean(staged_state, primed_state, step)

        # Uncache this and later steps since we just cleaned them: their status
        # has changed
        for current_step in [step] + step.next_steps():
            self._cache.clear_step(part, current_step)

        self._run_step(step=step, part=part, progress=progress, hint=hint)

    def _create_meta(self, step: steps.Step, part_names: Sequence[str]) -> None:
        if step == steps.PRIME and part_names == self.config.part_names:
            create_snap_packaging(self.config)

    def _handle_dirty(self, part, step, dirty_report, cli_config):
        dirty_action = cli_config.get_outdated_step_action()
        if not step.clean_if_dirty:
            if dirty_action == config.OutdatedStepAction.ERROR:
                raise errors.StepOutdatedError(
                    step=step, part=part.name, dirty_report=dirty_report
                )

        getattr(self, "_re{}".format(step.name))(
            part, hint="({})".format(dirty_report.get_summary())
        )

    def _handle_outdated(self, part, step, outdated_report, cli_config):
        dirty_action = cli_config.get_outdated_step_action()
        if not step.clean_if_dirty:
            if dirty_action == config.OutdatedStepAction.ERROR:
                raise errors.StepOutdatedError(
                    step=step, part=part.name, outdated_report=outdated_report
                )

        update_function = getattr(part, "update_{}".format(step.name), None)
        if update_function:
            self._prepare_step(step=step, part=part)
            notify_part_progress(
                part,
                "Updating {} step for".format(step.name),
                "({})".format(outdated_report.get_summary()),
            )
            update_function()

            # We know we just ran this step, so rather than check, manually
            # twiddle the cache
            self._complete_step(part, step)
        else:
            getattr(self, "_re{}".format(step.name))(
                part, "({})".format(outdated_report.get_summary())
            )


def notify_part_progress(part, progress, hint="", debug=False):
    if debug:
        logger.debug("%s %s %s", progress, part.name, hint)
    else:
        logger.info("%s %s %s", progress, part.name, hint)
