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
import tarfile
import time
from subprocess import check_call, Popen, PIPE, STDOUT
from tempfile import TemporaryDirectory

import yaml
from progressbar import AnimatedMarker, ProgressBar

import snapcraft
from snapcraft import formatting_utils
import snapcraft.internal
from snapcraft.internal import (
    common,
    lxd,
    meta,
    pluginhandler,
    repo,
    states
)
from snapcraft.internal import errors
from snapcraft.internal.cache import SnapCache
from snapcraft.internal.indicators import is_dumb_terminal
from snapcraft.internal.project_loader import replace_attr


logger = logging.getLogger(__name__)


_SNAPCRAFT_INTERNAL_DIR = os.path.join('snap', '.snapcraft')

_TEMPLATE_YAML = """name: my-snap-name # you probably want to 'snapcraft register <name>'
version: '0.1' # just for humans, typically '1.2+git' or '1.3.2'
summary: Single-line elevator pitch for your amazing snap # 79 char long summary
description: |
  This is my-snap's description. You have a paragraph or two to tell the
  most important story about your snap. Keep it under 100 words though,
  we live in tweetspace and your description wants to look good in the snap
  store.

grade: devel # must be 'stable' to release into candidate/stable channels
confinement: devmode # use 'strict' once you have the right plugs and slots

parts:
  my-part:
    # See 'snapcraft plugins'
    plugin: nil
"""  # noqa, lines too long.

_STEPS_TO_AUTOMATICALLY_CLEAN_IF_DIRTY = {'stage', 'prime'}


def init():
    """Initialize a snapcraft project."""
    snapcraft_yaml_path = os.path.join('snap', 'snapcraft.yaml')

    if os.path.exists(snapcraft_yaml_path):
        raise errors.SnapcraftEnvironmentError(
            '{} already exists!'.format(snapcraft_yaml_path))
    elif os.path.exists('snapcraft.yaml'):
        raise errors.SnapcraftEnvironmentError(
            'snapcraft.yaml already exists!')
    elif os.path.exists('.snapcraft.yaml'):
        raise errors.SnapcraftEnvironmentError(
            '.snapcraft.yaml already exists!')
    yaml = _TEMPLATE_YAML
    with contextlib.suppress(FileExistsError):
        os.mkdir(os.path.dirname(snapcraft_yaml_path))
    with open(snapcraft_yaml_path, mode='w') as f:
        f.write(yaml)

    return snapcraft_yaml_path


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
    config = snapcraft.internal.load_config(project_options)
    installed_packages = repo.Repo.install_build_packages(
        config.build_tools)
    if installed_packages is None:
        raise ValueError(
            'The repo backend is not returning the list of installed packages')

    installed_snaps = repo.snaps.install_snaps(config.build_snaps)

    os.makedirs(_SNAPCRAFT_INTERNAL_DIR, exist_ok=True)
    with open(os.path.join(_SNAPCRAFT_INTERNAL_DIR, 'state'), 'w') as f:
        f.write(yaml.dump(
            states.GlobalState(installed_packages, installed_snaps)))

    if (os.environ.get('SNAPCRAFT_SETUP_CORE') and
            config.data['confinement'] == 'classic'):
        _setup_core(project_options.deb_arch)

    _Executor(config, project_options).run(step, part_names)

    return {'name': config.data['name'],
            'version': config.data['version'],
            'arch': config.data['architectures'],
            'type': config.data.get('type', '')}


def _setup_core(deb_arch):
    core_path = common.get_core_path()
    if os.path.exists(core_path) and os.listdir(core_path):
        logger.debug('{!r} already exists, skipping core setup'.format(
            core_path))
        return
    snap_cache = SnapCache(project_name='snapcraft-core')

    # Try to get the latest revision.
    core_snap = snap_cache.get(deb_arch=deb_arch)
    if core_snap:
        # The current hash matches the filename
        current_hash = os.path.splitext(os.path.basename(core_snap))[0]
    else:
        current_hash = ''

    with TemporaryDirectory() as d:
        download_path = os.path.join(d, 'core.snap')
        download_hash = snapcraft.download('core', 'stable', download_path,
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
    check_call(['sudo', 'unsquashfs', '-d', core_path, core_snap])


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
        unstaged_prereqs = {p for p in prereqs
                            if 'stage' not in self._steps_run[p]}

        if unstaged_prereqs and not unstaged_prereqs.issubset(part_names):
            missing_parts = [part_name for part_name in self.config.part_names
                             if part_name in unstaged_prereqs]
            if missing_parts:
                raise RuntimeError(
                    'Requested {!r} of {!r} but there are unsatisfied '
                    'prerequisites: {!r}'.format(
                        step, part.name, ' '.join(missing_parts)))
        elif unstaged_prereqs:
            # prerequisites need to build all the way to the staging
            # step to be able to share the common assets that make them
            # a dependency.
            logger.info(
                '{!r} has prerequisites that need to be staged: '
                '{}'.format(part.name, ' '.join(unstaged_prereqs)))
            self.run('stage', unstaged_prereqs)

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
                self.config.data, self.project_options,
                self.config.snapcraft_yaml_path)

    def _handle_dirty(self, part, step, dirty_report):
        if step not in _STEPS_TO_AUTOMATICALLY_CLEAN_IF_DIRTY:
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


def _create_tar_filter(tar_filename):
    def _tar_filter(tarinfo):
        fn = tarinfo.name
        if fn.startswith('./parts/') and not fn.startswith('./parts/plugins'):
            return None
        elif fn in ('./stage', './prime', tar_filename):
            return None
        elif fn.endswith('.snap'):
            return None
        return tarinfo
    return _tar_filter


def containerbuild(step, project_options, container_config,
                   output=None, args=[]):
    config = snapcraft.internal.load_config(project_options)
    if container_config.remote:
        logger.info('Using LXD remote {!r} from SNAPCRAFT_CONTAINER_BUILDS'
                    .format(container_config.remote))
    else:
        logger.info('Using default LXD remote because '
                    'SNAPCRAFT_CONTAINER_BUILDS is set to 1')
    lxd.Project(output=output, source=os.path.curdir,
                project_options=project_options,
                remote=container_config.remote,
                metadata=config.get_metadata()).execute(step, args)


def cleanbuild(project_options, remote=''):
    if remote and not lxd._remote_is_valid(remote):
        raise errors.InvalidContainerRemoteError(remote)

    config = snapcraft.internal.load_config(project_options)
    tar_filename = '{}_{}_source.tar.bz2'.format(
        config.data['name'], config.data['version'])

    with tarfile.open(tar_filename, 'w:bz2') as t:
        t.add(os.path.curdir, filter=_create_tar_filter(tar_filename))
    lxd.Cleanbuilder(source=tar_filename,
                     project_options=project_options,
                     metadata=config.get_metadata(), remote=remote).execute()


def _snap_data_from_dir(directory):
    with open(os.path.join(directory, 'meta', 'snap.yaml')) as f:
        snap = yaml.load(f)

    return {'name': snap['name'],
            'version': snap['version'],
            'arch': snap.get('architectures', []),
            'type': snap.get('type', '')}


def snap(project_options, directory=None, output=None):
    if not directory:
        directory = project_options.prime_dir
        execute('prime', project_options)

    return pack(directory, output)


def pack(directory, output=None):
    # Check for our prerequesite external command early
    repo.check_for_command('mksquashfs')

    snap = _snap_data_from_dir(directory)
    snap_name = output or common.format_snap_name(snap)

    # If a .snap-build exists at this point, when we are about to override
    # the snap blob, it is stale. We rename it so user have a chance to
    # recover accidentally lost assertions.
    snap_build = snap_name + '-build'
    if os.path.isfile(snap_build):
        _new = '{}.{}'.format(snap_build, int(time.time()))
        logger.warning('Renaming stale build assertion to {}'.format(_new))
        os.rename(snap_build, _new)

    # These options need to match the review tools:
    # http://bazaar.launchpad.net/~click-reviewers/click-reviewers-tools/trunk/view/head:/clickreviews/common.py#L38
    mksquashfs_args = ['-noappend', '-comp', 'xz', '-no-xattrs']
    if snap['type'] != 'os':
        mksquashfs_args.append('-all-root')

    with Popen(['mksquashfs', directory, snap_name] + mksquashfs_args,
               stdout=PIPE, stderr=STDOUT) as proc:
        ret = None
        if is_dumb_terminal():
            logger.info('Snapping {!r} ...'.format(snap['name']))
            ret = proc.wait()
        else:
            message = '\033[0;32m\rSnapping {!r}\033[0;32m '.format(
                snap['name'])
            progress_indicator = ProgressBar(
                widgets=[message, AnimatedMarker()], maxval=7)
            progress_indicator.start()

            ret = proc.poll()
            count = 0

            while ret is None:
                if count >= 7:
                    progress_indicator.start()
                    count = 0
                progress_indicator.update(count)
                count += 1
                time.sleep(.2)
                ret = proc.poll()
        print('')
        if ret != 0:
            logger.error(proc.stdout.read().decode('utf-8'))
            raise RuntimeError('Failed to create snap {!r}'.format(snap_name))

        logger.debug(proc.stdout.read().decode('utf-8'))

    return snap_name


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
    if os.path.exists(_SNAPCRAFT_INTERNAL_DIR):
        shutil.rmtree(_SNAPCRAFT_INTERNAL_DIR)


def clean(project_options, parts, step=None):
    # step defaults to None because that's how it comes from docopt when it's
    # not set.
    if not step:
        step = 'pull'

    if not parts and step == 'pull':
        _cleanup_common_directories_for_step(step, project_options)
        return

    config = snapcraft.internal.load_config()

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
