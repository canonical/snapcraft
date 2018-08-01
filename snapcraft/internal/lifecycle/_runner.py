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

import logging
import os
from subprocess import check_call
from tempfile import TemporaryDirectory
from typing import Sequence

import snapcraft
from snapcraft import config
from snapcraft.internal import (
    common,
    errors,
    meta,
    pluginhandler,
    project_loader,
    repo,
    states,
    steps,
)
from snapcraft.internal.cache import SnapCache
from ._status_cache import StatusCache


logger = logging.getLogger(__name__)


def execute(
    step: steps.Step,
    project_config: "snapcraft.internal.project_loader._config.Config",
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
    installed_packages = repo.Repo.install_build_packages(project_config.build_tools)
    if installed_packages is None:
        raise ValueError(
            "The repo backend is not returning the list of installed packages"
        )

    installed_snaps = repo.snaps.install_snaps(project_config.build_snaps)

    try:
        global_state = states.GlobalState.load(
            filepath=project_config.project._global_state_file
        )
    except FileNotFoundError:
        global_state = states.GlobalState()
    global_state.append_build_packages(installed_packages)
    global_state.append_build_snaps(installed_snaps)
    global_state.save(filepath=project_config.project._global_state_file)

    if _should_get_core(project_config.data.get("confinement")):
        _setup_core(
            project_config.project.deb_arch, project_config.data.get("base", "core")
        )

    executor = _Executor(project_config)
    executor.run(step, part_names)
    if not executor.steps_were_run:
        logger.warn(
            "The requested action has already been taken. Consider\n"
            "specifying parts, or clean the steps you want to run again."
        )

    return {
        "name": project_config.data["name"],
        "version": project_config.data.get("version"),
        "arch": project_config.data["architectures"],
        "type": project_config.data.get("type", ""),
    }


def _setup_core(deb_arch, base):
    core_path = common.get_core_path(base)
    if os.path.exists(core_path) and os.listdir(core_path):
        logger.debug("{!r} already exists, skipping core setup".format(core_path))
        return

    # for backwards compatibility
    if base == "core":
        snap_cache = SnapCache(project_name="snapcraft-core")
    else:
        snap_cache = SnapCache(project_name=base)

    # Try to get the latest revision.
    core_snap = snap_cache.get(deb_arch=deb_arch)
    if core_snap:
        # The current hash matches the filename
        current_hash = os.path.splitext(os.path.basename(core_snap))[0]
    else:
        current_hash = ""

    with TemporaryDirectory() as d:
        download_path = os.path.join(d, "{}.snap".format(base))
        download_hash = snapcraft.download(
            base, "stable", download_path, deb_arch, except_hash=current_hash
        )
        if download_hash != current_hash:
            snap_cache.cache(snap_filename=download_path)
            snap_cache.prune(deb_arch=deb_arch, keep_hash=download_hash)

    core_snap = snap_cache.get(deb_arch=deb_arch)

    # Now unpack
    logger.info("Setting up {!r} in {!r}".format(core_snap, core_path))
    if os.path.exists(core_path) and not os.listdir(core_path):
        check_call(["sudo", "rmdir", core_path])
    check_call(["sudo", "mkdir", "-p", os.path.dirname(core_path)])
    unsquashfs_path = snapcraft.file_utils.get_tool_path("unsquashfs")
    check_call(["sudo", unsquashfs_path, "-d", core_path, core_snap])


def _should_get_core(confinement: str) -> bool:
    is_env_var_set = os.environ.get("SNAPCRAFT_SETUP_CORE", False) is not False
    # This is a quirk so that docker users not using the Dockerfile
    # we distribute and create can automatically build classic
    is_docker_instance = common.is_docker_instance()  # type: bool
    is_classic = confinement == "classic"  # type: bool

    return is_classic and (is_env_var_set or is_docker_instance)


def _replace_in_part(part):
    for key, value in part.plugin.options.__dict__.items():
        replacements = project_loader.environment_to_replacements(
            project_loader.snapcraft_part_environment(part)
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

    def _create_meta(self, step, part_names):
        if step == steps.PRIME and part_names == self.config.part_names:
            common.env = self.config.snap_env()
            meta.create_snap_packaging(
                self.config.data,
                self.config.parts,
                self.project,
                self.config.validator.schema,
            )

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
