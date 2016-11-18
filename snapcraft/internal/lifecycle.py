# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
from subprocess import Popen, PIPE, STDOUT

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
)
from snapcraft.internal.indicators import is_dumb_terminal
from snapcraft.internal.project_loader import replace_attr


logger = logging.getLogger(__name__)


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

    if os.path.exists('snapcraft.yaml'):
        raise EnvironmentError('snapcraft.yaml already exists!')
    elif os.path.exists('.snapcraft.yaml'):
        raise EnvironmentError('.snapcraft.yaml already exists!')
    yaml = _TEMPLATE_YAML.strip()
    with open('snapcraft.yaml', mode='w+') as f:
        f.write(yaml)
    logger.info('Created snapcraft.yaml.')
    logger.info(
        'Edit the file to your liking or run `snapcraft` to get started')


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
    repo.install_build_packages(config.build_tools)

    _Executor(config, project_options).run(step, part_names)

    return {'name': config.data['name'],
            'version': config.data['version'],
            'arch': config.data['architectures'],
            'type': config.data.get('type', '')}


def _replace_in_part(part):
    for key, value in part.code.options.__dict__.items():
        value = replace_attr(value, [
            ('$SNAPCRAFT_PART_INSTALL', part.code.installdir),
        ])
        setattr(part.code.options, key, value)

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
                if part.is_dirty(step):
                    self._handle_dirty(part, step)
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
            meta.create_snap_packaging(self.config.data,
                                       self.project_options.snap_dir,
                                       self.project_options.parts_dir)

    def _handle_dirty(self, part, step):
        if step not in _STEPS_TO_AUTOMATICALLY_CLEAN_IF_DIRTY:
            raise RuntimeError(
                'The {0!r} step of {1!r} is out of date. Please clean that '
                "part's {0!r} step in order to rebuild".format(
                    step, part.name))

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
                    humanized_parts = formatting_utils.humanize_list(
                        dependents, 'and')

                    raise RuntimeError(
                        'The {0!r} step for {1!r} needs to be run again, but '
                        '{2} depend{3} upon it. Please clean the build '
                        'step of {2} first.'.format(
                            step, part.name, humanized_parts,
                            's' if len(dependents) == 1 else ''))

        part.clean(staged_state, primed_state, step, '(out of date)')


def _create_tar_filter(tar_filename):
    def _tar_filter(tarinfo):
        fn = tarinfo.name
        if fn.startswith('./parts/') and not fn.startswith('./parts/plugins'):
            return None
        elif fn in ('./stage', './prime', './snap', tar_filename):
            return None
        elif fn.endswith('.snap'):
            return None
        return tarinfo
    return _tar_filter


def cleanbuild(project_options):
    if not repo.is_package_installed('lxd'):
        raise EnvironmentError(
            'The lxd package is not installed, in order to use `cleanbuild` '
            'you must install lxd onto your system. Refer to the '
            '"Ubuntu Desktop and Ubuntu Server" section on '
            'https://linuxcontainers.org/lxd/getting-started-cli/'
            '#ubuntu-desktop-and-ubuntu-server to enable a proper setup.')

    config = snapcraft.internal.load_config(project_options)
    tar_filename = '{}_{}_source.tar.bz2'.format(
        config.data['name'], config.data['version'])

    with tarfile.open(tar_filename, 'w:bz2') as t:
        t.add(os.path.curdir, filter=_create_tar_filter(tar_filename))

    snap_filename = common.format_snap_name(config.data)
    lxd.Cleanbuilder(snap_filename, tar_filename, project_options).execute()


def _snap_data_from_dir(directory):
    with open(os.path.join(directory, 'meta', 'snap.yaml')) as f:
        snap = yaml.load(f)

    return {'name': snap['name'],
            'version': snap['version'],
            'arch': snap.get('architectures', []),
            'type': snap.get('type', '')}


def snap(project_options, directory=None, output=None):
    if directory:
        snap_dir = os.path.abspath(directory)
        snap = _snap_data_from_dir(snap_dir)
    else:
        # make sure the full lifecycle is executed
        snap_dir = project_options.snap_dir
        snap = execute('prime', project_options)

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

    with Popen(['mksquashfs', snap_dir, snap_name] + mksquashfs_args,
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

    logger.info('Snapped {}'.format(snap_name))


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

    # Verify that they're either already clean, or that they will be cleaned.
    if not dependents.issubset(clean_part_names):
        for part in config.all_parts:
            if part.name in dependents and not part.is_clean(step):
                humanized_parts = formatting_utils.humanize_list(
                    dependents, 'and')

                raise RuntimeError(
                    'Requested clean of {!r} but {} depend{} upon it. Please '
                    "add each to the clean command if that's what you "
                    'intended.'.format(part_name, humanized_parts,
                                       's' if len(dependents) == 1 else ''))


def _clean_parts(part_names, step, config, staged_state, primed_state):
    if not step:
        step = 'pull'

    # Before doing anything, verify that we weren't asked to clean only the
    # root of a dependency tree (the entire tree must be specified).
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
    _remove_directory_if_empty(project_options.parts_dir)
    _remove_directory_if_empty(project_options.stage_dir)
    _remove_directory_if_empty(project_options.snap_dir)

    max_index = -1
    for part in config.all_parts:
        step = part.last_step()
        if step:
            index = common.COMMAND_ORDER.index(step)
            if index > max_index:
                max_index = index

    # If no parts have been pulled, remove the parts directory. In most cases
    # this directory should have already been cleaned, but this handles the
    # case of a failed pull. Note however that the presence of local plugins
    # should prevent this removal.
    if (max_index < common.COMMAND_ORDER.index('pull') and
            os.path.exists(project_options.parts_dir) and not
            os.path.exists(project_options.local_plugins_dir)):
        logger.info('Cleaning up parts directory')
        shutil.rmtree(project_options.parts_dir)

    # If no parts have been staged, remove staging area.
    should_remove_stagedir = max_index < common.COMMAND_ORDER.index('stage')
    if should_remove_stagedir and os.path.exists(project_options.stage_dir):
        logger.info('Cleaning up staging area')
        shutil.rmtree(project_options.stage_dir)

    # If no parts have been primed, remove snapping area.
    should_remove_snapdir = max_index < common.COMMAND_ORDER.index('prime')
    if should_remove_snapdir and os.path.exists(project_options.snap_dir):
        logger.info('Cleaning up snapping area')
        shutil.rmtree(project_options.snap_dir)


def clean(project_options, parts, step=None):
    config = snapcraft.internal.load_config()

    if parts:
        config.parts.validate(parts)
    else:
        parts = [part.name for part in config.all_parts]

    staged_state = config.get_project_state('stage')
    primed_state = config.get_project_state('prime')

    _clean_parts(parts, step, config, staged_state, primed_state)

    _cleanup_common_directories(config, project_options)
