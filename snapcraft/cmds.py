# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import apt
import filecmp
import logging
import os
import shlex
import shutil
import subprocess
import sys
import time

import snapcraft.yaml
from snapcraft import (
    common,
    meta,
)

logger = logging.getLogger(__name__)


_TEMPLATE_YAML = r'''name: # the name of the snap
version: # the version of the snap
# The vendor for the snap (replace 'Vendor <email@example.com>')
vendor: Vendor <email@example.com>
summary: # 79 char long summary
description: # A longer description for the snap
icon: # A path to an icon for the package
'''


def init(args):
    if os.path.exists('snapcraft.yaml'):
        logger.error('snapcraft.yaml already exists!')
        sys.exit(1)
    yaml = _TEMPLATE_YAML.strip()
    with open('snapcraft.yaml', mode='w+') as f:
        f.write(yaml)
    logger.info('Wrote the following as snapcraft.yaml.')
    print()
    print(yaml)
    sys.exit(0)


def shell(args):
    config = snapcraft.yaml.load_config()
    common.env = config.stage_env()
    userCommand = args.userCommand
    if not userCommand:
        userCommand = ['/usr/bin/env',
                       'PS1=\[\e[1;32m\]snapcraft:\w\$\[\e[0m\] ',
                       '/bin/bash',
                       '--norc']
    common.run(userCommand)


def snap(args):
    cmd(args)

    config = snapcraft.yaml.load_config()
    # TODO move all this to meta.create
    if 'architectures' in config.data:
        arches = config.data['architectures']
    else:
        arches = [snapcraft.common.get_arch(), ]

    # FIXME this should be done in a more contained manner
    common.env = config.snap_env()

    meta.create(config.data, arches)


def assemble(args):
    args.cmd = 'snap'
    # With all the data in snapcraft.yaml, maybe it's not a good idea to call
    # snap(args) and just do a snappy build if assemble was explicitly called.
    snap(args)

    ticker = '/-\\|'
    i = 0
    with subprocess.Popen(['snappy', 'build', common.get_snapdir()],
                          stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE,) as proc:
        ret = None
        if os.isatty(sys.stdout.fileno()):
            ret = proc.poll()
            while ret is None:
                print('\033[1m\rSnapping\033[0m {}'.format(ticker[i]), end='')
                i = (i+1) % len(ticker)
                time.sleep(.2)
                ret = proc.poll()
        else:
            print('Snapping ...')
            ret = proc.wait()
        print()
        if ret == 0:
            print(proc.stdout.read().decode('utf-8'))
        else:
            print(proc.stderr.read().decode('utf-8'), file=sys.stderr)
        sys.exit(ret)


def _check_for_collisions(parts):
    parts_files = {}
    for part in parts:
        # Gather our own files up
        part_files, _ = part.migratable_fileset_for('stage')

        # Scan previous parts for collisions
        for other_part_name in parts_files:
            common = part_files & parts_files[other_part_name]['files']
            conflict_files = []
            for f in common:
                this = os.path.join(part.installdir, f)
                other = os.path.join(
                    parts_files[other_part_name]['installdir'],
                    f)
                if os.path.islink(this) and os.path.islink(other):
                    continue
                if not filecmp.cmp(this, other, shallow=False):
                    conflict_files.append(f)

            if conflict_files:
                logger.error('Error: parts %s and %s have the following file '
                             'paths in common which have different '
                             'contents:\n  %s',
                             other_part_name,
                             part.name,
                             '\n  '.join(sorted(conflict_files)))

                return False

        # And add our files to the list
        parts_files[part.name] = {'files': part_files,
                                  'installdir': part.installdir}

    return True


def cmd(args):
    forceAll = args.force
    forceCommand = None

    cmds = [args.cmd]

    if cmds[0] in common.COMMAND_ORDER:
        forceCommand = cmds[0]
        cmds = common.COMMAND_ORDER[0:common.COMMAND_ORDER.index(cmds[0]) + 1]

    config = snapcraft.yaml.load_config()
    _install_build_packages(config.build_tools)

    # clean the snap dir before Snapping
    snap_clean = False

    for part in config.all_parts:
        for cmd in cmds:
            if cmd is 'stage':
                # This ends up running multiple times, as each part gets to its
                # staging cmd.  That's inefficient, but largely OK.
                # FIXME: fix the above by iterating over cmds before iterating
                # all_parts.  But then we need to make sure we continue to
                # handle cases like go, where you want go built before trying
                # to pull a go project.
                if not _check_for_collisions(config.all_parts):
                    sys.exit(1)

            # We want to make sure we have a clean snap dir
            if cmd is 'snap' and not snap_clean:
                shutil.rmtree(common.get_snapdir())
                snap_clean = True

            common.env = config.build_env_for_part(part)
            force = forceAll or cmd == forceCommand

            try:
                getattr(part, cmd)(force=force)
            except Exception as e:
                logger.error('Failed doing %s for %s: %s', cmd, part.name, e)
                sys.exit(1)


def _call(args, **kwargs):
    logger.info('Running: %s', ' '.join(shlex.quote(arg) for arg in args))
    return subprocess.call(args, **kwargs)


def _check_call(args, **kwargs):
    logger.info('Running: %s', ' '.join(shlex.quote(arg) for arg in args))
    return subprocess.check_call(args, **kwargs)


def _install_build_packages(packages):
    new_packages = []
    for pkg in packages:
        try:
            if not apt.Cache()[pkg].installed:
                new_packages.append(pkg)
        except KeyError:
            logger.error('Could not find all the "build-packages" required '
                         'in snapcraft.yaml')
            sys.exit(1)
    if new_packages:
        logger.info('Installing required packages on the host system')
        _check_call(['sudo', 'apt-get', '-o', 'Dpkg::Progress-Fancy=1',
                     '--no-install-recommends',
                     '-y', 'install'] + new_packages)
