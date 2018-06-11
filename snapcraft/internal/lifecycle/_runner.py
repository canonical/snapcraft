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
import collections
import logging
import os
from subprocess import check_call
from tempfile import TemporaryDirectory

import yaml
from tabulate import tabulate

import snapcraft
from snapcraft import config, formatting_utils
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
from . import constants
from ._clean import mark_dependents_dirty


logger = logging.getLogger(__name__)


def execute(step: steps.Step, project_options, part_names=None):
    """Execute until step in the lifecycle for part_names or all parts.

    Lifecycle execution will happen for each step iterating over all
    the available parts, if part_names is specified, only those parts
    will run.

    If one of the parts to execute has an after keyword, execution is
    forced until the stage step for such part. If part_names was provided
    and after is not in this set, an exception will be raised.

    :param str step: A valid step in the lifecycle: pull, build, prime or snap.
    :param project_options: Runtime options for the project.
    :param list part_names: A list of parts to execute the lifecycle on.
    :raises RuntimeError: If a prerequesite of the part needs to be staged
                          and such part is not in the list of parts to iterate
                          over.
    :returns: A dict with the snap name, version, type and architectures.
    """
    config = project_loader.load_config(project_options)
    installed_packages = repo.Repo.install_build_packages(
        config.build_tools)
    if installed_packages is None:
        raise ValueError(
            'The repo backend is not returning the list of installed packages')

    installed_snaps = repo.snaps.install_snaps(config.build_snaps)

    os.makedirs(constants.SNAPCRAFT_INTERNAL_DIR, exist_ok=True)
    state_path = os.path.join(constants.SNAPCRAFT_INTERNAL_DIR, 'state')
    with open(state_path, 'w') as state_file:
        state_file.write(yaml.dump(
            states.GlobalState(installed_packages, installed_snaps)))

    if _should_get_core(config.data.get('confinement')):
        _setup_core(project_options.deb_arch,
                    config.data.get('base', 'core'))

    executor = _Executor(config, project_options)
    executor.run(step, part_names)
    if not executor.steps_were_run:
        logger.warn(
            'The requested action has already been taken. Consider\n'
            'specifying parts, or clean the steps you want to run again.')

    return {'name': config.data['name'],
            'version': config.data.get('version'),
            'arch': config.data['architectures'],
            'type': config.data.get('type', '')}


def _setup_core(deb_arch, base):
    core_path = common.get_core_path(base)
    if os.path.exists(core_path) and os.listdir(core_path):
        logger.debug('{!r} already exists, skipping core setup'.format(
            core_path))
        return

    # for backwards compatibility
    if base == 'core':
        snap_cache = SnapCache(project_name='snapcraft-core')
    else:
        snap_cache = SnapCache(project_name=base)

    # Try to get the latest revision.
    core_snap = snap_cache.get(deb_arch=deb_arch)
    if core_snap:
        # The current hash matches the filename
        current_hash = os.path.splitext(os.path.basename(core_snap))[0]
    else:
        current_hash = ''

    with TemporaryDirectory() as d:
        download_path = os.path.join(d, '{}.snap'.format(base))
        download_hash = snapcraft.download(base, 'stable', download_path,
                                           deb_arch, except_hash=current_hash)
        if download_hash != current_hash:
            snap_cache.cache(snap_filename=download_path)
            snap_cache.prune(deb_arch=deb_arch, keep_hash=download_hash)

    core_snap = snap_cache.get(deb_arch=deb_arch)

    # Now unpack
    logger.info('Setting up {!r} in {!r}'.format(core_snap, core_path))
    if os.path.exists(core_path) and not os.listdir(core_path):
        check_call(['sudo', 'rmdir', core_path])
    check_call(['sudo', 'mkdir', '-p', os.path.dirname(core_path)])
    unsquashfs_path = snapcraft.file_utils.get_tool_path('unsquashfs')
    check_call(['sudo', unsquashfs_path, '-d', core_path, core_snap])


def _should_get_core(confinement: str) -> bool:
    is_env_var_set = os.environ.get('SNAPCRAFT_SETUP_CORE', False) is not False
    # This is a quirk so that docker users not using the Dockerfile
    # we distribute and create can automatically build classic
    is_docker_instance = common.is_docker_instance()  # type: bool
    is_classic = (confinement == 'classic')  # type: bool

    return is_classic and (is_env_var_set or is_docker_instance)


def _replace_in_part(part):
    for key, value in part.plugin.options.__dict__.items():
        replacements = project_loader.environment_to_replacements(
            project_loader.snapcraft_part_environment(part))

        value = project_loader.replace_attr(value, replacements)
        setattr(part.plugin.options, key, value)

    return part


class _Executor:

    def __init__(self, config, project_options):
        self.config = config
        self.project_options = project_options
        self.parts_config = config.parts
        self.steps_were_run = False
        self._steps_run = collections.defaultdict(set)
        self._dirty_reports = collections.defaultdict(dict)
        summary = []

        # Initialize steps run and dirty reports, so we only need to fetch
        # them once
        for part in self.config.all_parts:
            part_summary = {'part': part.name}
            for step in steps.STEPS:
                part_summary[step] = None
                self._dirty_reports[part.name][step] = part.get_dirty_report(
                    step)
                if not part.should_step_run(step):
                    self._steps_run[part.name].add(step)
                    part_summary[step] = 'complete'
                if self._dirty_reports[part.name][step]:
                    part_summary[step] = 'dirty'
            summary.append(part_summary)

        # Print summary in debug mode
        logger.debug(tabulate(summary, headers="keys"))

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
                    if self._dirty_reports[part.name][current_step]:
                        self._handle_dirty(
                            part, current_step,
                            self._dirty_reports[part.name][current_step],
                            cli_config)
                    elif current_step in self._steps_run[part.name]:
                        # By default, if a step has already run, don't run it
                        # again. However, automatically clean and re-run the
                        # step if all the following conditions apply:
                        #
                        #   1. The step is the exact step that was requested
                        #      (not an earlier one)
                        #   2. The part was explicitly specified
                        if (part_names and current_step == step and
                                part.name in part_names):
                            getattr(self, '_re{}'.format(
                                current_step.name))(part)
                        else:
                            notify_part_progress(
                                part,
                                'Skipping {}'.format(current_step.name),
                                '(already ran)')
                    else:
                        getattr(self, '_run_{}'.format(
                            current_step.name))(part)
                        self._steps_run[part.name].add(current_step)

        self._create_meta(step, processed_part_names)

    def _run_pull(self, part):
        self._run_step(step=steps.PULL, part=part, progress='Pulling')

    def _repull(self, part, hint=''):
        self._rerun_step(
            step=steps.PULL, part=part,
            progress='Cleaning later steps and re-pulling', hint=hint)

    def _run_build(self, part):
        self._run_step(step=steps.BUILD, part=part, progress='Building')

    def _rebuild(self, part, hint=''):
        self._rerun_step(
            step=steps.BUILD, part=part,
            progress='Cleaning later steps and re-building', hint=hint)

    def _run_stage(self, part):
        self._run_step(step=steps.STAGE, part=part, progress='Staging')

    def _restage(self, part, hint=''):
        self._rerun_step(
            step=steps.STAGE, part=part,
            progress='Cleaning later steps and re-staging', hint=hint)

    def _run_prime(self, part):
        self._run_step(
            step=steps.PRIME, part=part, progress='Priming',
            prerequisite_step=steps.PRIME)

    def _reprime(self, part, hint=''):
        self._rerun_step(
            step=steps.PRIME, part=part, progress='Re-priming', hint=hint,
            prerequisite_step=steps.PRIME)

    def _run_step(self, *, step: steps.Step, part, progress, hint='',
                  prerequisite_step: steps.Step=steps.STAGE):
        common.reset_env()
        prereqs = self.parts_config.get_prereqs(part.name)

        step_prereqs = {p for p in prereqs
                        if prerequisite_step not in self._steps_run[p]}

        if step_prereqs:
            # prerequisites need to go all the way to the prerequisite step to
            # be able to share the common assets that make them a dependency.
            logger.info(
                '{!r} has prerequisites that need to be {}d: {}'.format(
                    part.name, prerequisite_step.name, ' '.join(step_prereqs)))
            self.run(prerequisite_step, step_prereqs)

        # Run the preparation function for this step (if implemented)
        preparation_function = getattr(
            part, 'prepare_{}'.format(step.name), None)
        if preparation_function:
            notify_part_progress(
                part, 'Preparing to {}'.format(step.name), debug=True)
            preparation_function()

        common.env = self.parts_config.build_env_for_part(part)
        common.env.extend(self.config.project_env())

        part = _replace_in_part(part)

        notify_part_progress(part, progress, hint)
        getattr(part, step.name)()
        self._steps_run[part.name].add(step)
        self.steps_were_run = True

    def _rerun_step(self, *, step: steps.Step, part, progress, hint='',
                    prerequisite_step: steps.Step=steps.STAGE):
        staged_state = self.config.get_project_state(steps.STAGE)
        primed_state = self.config.get_project_state(steps.PRIME)

        # We need to clean this step, but if it involves cleaning any steps
        # upon which other parts depend, those parts need to be marked as dirty
        # so they can run again, taking advantage of the new dependency.
        dirty_parts = mark_dependents_dirty(part.name, step, self.config)

        # First clean the step, then run it again
        part.clean(staged_state, primed_state, step)

        for current_step in [step] + step.next_steps():
            self._dirty_reports[part.name][current_step] = None
            self._steps_run[part.name].discard(current_step)

        self._run_step(
            step=step, part=part, progress=progress, hint=hint,
            prerequisite_step=prerequisite_step)

        if dirty_parts:
            logger.warning(
                'The following {} now out of date: {}'.format(
                    formatting_utils.pluralize(
                        dirty_parts, 'part is', 'parts are'),
                    formatting_utils.humanize_list(dirty_parts, 'and')))

    def _create_meta(self, step, part_names):
        if step == steps.PRIME and part_names == self.config.part_names:
            common.env = self.config.snap_env()
            meta.create_snap_packaging(
                self.config.data, self.config.parts, self.project_options,
                self.config.snapcraft_yaml_path,
                self.config.original_snapcraft_yaml,
                self.config.validator.schema)

    def _handle_dirty(self, part, step, dirty_report, cli_config):
        dirty_action = cli_config.get_outdated_step_action()
        if not step.clean_if_dirty:
            if dirty_action == config.OutdatedStepAction.ERROR:
                raise errors.StepOutdatedError(
                    step=step, part=part.name,
                    dirty_properties=dirty_report.dirty_properties,
                    dirty_project_options=dirty_report.dirty_project_options,
                    changed_dependencies=dirty_report.changed_dependencies)

        getattr(self, '_re{}'.format(step.name))(part, hint='(out of date)')


def notify_part_progress(part, progress, hint='', debug=False):
    if debug:
        logger.debug('%s %s %s', progress, part.name, hint)
    else:
        logger.info('%s %s %s', progress, part.name, hint)
