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
from subprocess import check_call
from tempfile import TemporaryDirectory

import yaml

import snapcraft
from snapcraft.internal import (
    common,
    meta,
    pluginhandler,
    project_loader,
    repo,
    states,
)
from snapcraft.internal import errors
from snapcraft.internal.cache import SnapCache
from snapcraft.internal.project_loader import replace_attr
from . import constants


logger = logging.getLogger(__name__)


def execute(step, project_options, part_names=None):
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

    _Executor(config, project_options).run(step, part_names)

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
        value = replace_attr(value, [
            ('$SNAPCRAFT_PART_INSTALL', part.plugin.installdir),
        ])
        setattr(part.plugin.options, key, value)

    return part


class _Executor:

    def __init__(self, config, project_options):
        self.config = config
        self.project_options = project_options
        self.parts_config = config.parts
        self._steps_run = self._init_run_states()

    def _init_run_states(self):
        steps_run = {}

        for part in self.config.all_parts:
            steps_run[part.name] = set()
            for step in common.COMMAND_ORDER:
                dirty_report = part.get_dirty_report(step)
                if dirty_report:
                    self._handle_dirty(part, step, dirty_report)
                elif not (part.should_step_run(step)):
                    steps_run[part.name].add(step)
                    part.notify_part_progress('Skipping {}'.format(step),
                                              '(already ran)')

        return steps_run

    def run(self, step, part_names=None):
        if part_names:
            self.parts_config.validate(part_names)
            # self.config.all_parts is already ordered, let's not lose that
            # and keep using a list.
            parts = [p for p in self.config.all_parts if p.name in part_names]
        else:
            parts = self.config.all_parts
            part_names = self.config.part_names

        step_index = common.COMMAND_ORDER.index(step) + 1

        for step in common.COMMAND_ORDER[0:step_index]:
            if step == 'stage':
                # XXX check only for collisions on the parts that have already
                # been built --elopio - 20170713
                pluginhandler.check_for_collisions(self.config.all_parts)
            for part in parts:
                if step not in self._steps_run[part.name]:
                    self._run_step(step, part, part_names)
                    self._steps_run[part.name].add(step)

        self._create_meta(step, part_names)

    def _run_step(self, step, part, part_names):
        common.reset_env()
        prereqs = self.parts_config.get_prereqs(part.name)

        # Dependencies need to be primed to have paths to.
        if step == 'prime':
            required_step = 'prime'
        # Or staged to be built with.
        else:
            required_step = 'stage'

        step_prereqs = {p for p in prereqs
                        if required_step not in self._steps_run[p]}

        if step_prereqs and not step_prereqs.issubset(part_names):
            missing_parts = [part_name for part_name in self.config.part_names
                             if part_name in step_prereqs]
            if missing_parts:
                raise RuntimeError(
                    'Requested {!r} of {!r} but there are unsatisfied '
                    'prerequisites: {!r}'.format(
                        step, part.name, ' '.join(missing_parts)))
        elif step_prereqs:
            # prerequisites need to build all the way to the staging
            # step to be able to share the common assets that make them
            # a dependency.
            logger.info(
                '{!r} has prerequisites that need to be {}d: '
                '{}'.format(part.name, required_step, ' '.join(step_prereqs)))
            self.run(required_step, step_prereqs)

        # Run the preparation function for this step (if implemented)
        with contextlib.suppress(AttributeError):
            getattr(part, 'prepare_{}'.format(step))()

        common.env = self.parts_config.build_env_for_part(part)
        common.env.extend(self.config.project_env())

        part = _replace_in_part(part)

        getattr(part, step)()

    def _create_meta(self, step, part_names):
        if step == 'prime' and part_names == self.config.part_names:
            common.env = self.config.snap_env()
            meta.create_snap_packaging(
                self.config.data, self.config.parts, self.project_options,
                self.config.snapcraft_yaml_path,
                self.config.original_snapcraft_yaml,
                self.config.validator.schema)

    def _handle_dirty(self, part, step, dirty_report):
        if step not in constants.STEPS_TO_AUTOMATICALLY_CLEAN_IF_DIRTY:
            raise errors.StepOutdatedError(
                step=step, part=part.name,
                dirty_properties=dirty_report.dirty_properties,
                dirty_project_options=dirty_report.dirty_project_options)

        staged_state = self.config.get_project_state('stage')
        primed_state = self.config.get_project_state('prime')

        # We need to clean this step, but if it involves cleaning the stage
        # step and it has dependents that have been built, we need to ask for
        # them to first be cleaned (at least back to the build step).
        index = common.COMMAND_ORDER.index(step)
        dependents = self.parts_config.get_dependents(part.name)
        if (index <= common.COMMAND_ORDER.index('stage') and
                not part.is_clean('stage') and dependents):
            for dependent in self.config.all_parts:
                if (dependent.name in dependents and
                        not dependent.is_clean('build')):
                    raise errors.StepOutdatedError(step=step, part=part.name,
                                                   dependents=dependents)

        part.clean(staged_state, primed_state, step, '(out of date)')
