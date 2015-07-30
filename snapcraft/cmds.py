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

import glob
import logging
import os
import shlex
import subprocess
import sys
import tempfile
import time

import yaml

import snapcraft.plugin
import snapcraft.yaml
from snapcraft import common


logger = logging.getLogger(__name__)


def init(args):
    if os.path.exists("snapcraft.yaml"):
        logger.error('snapcraft.yaml already exists!')
        sys.exit(1)
    yaml = 'parts:\n'
    for part_name in args.part:
        part = snapcraft.plugin.load_plugin(part_name, part_name, load_code=False)
        yaml += '    ' + part.names()[0] + ':\n'
        for opt in part.config.get('options', []):
            if part.config['options'][opt].get('required', False):
                yaml += '        ' + opt + ':\n'
    yaml = yaml.strip()
    with open('snapcraft.yaml', mode='w+') as f:
        f.write(yaml)
    logger.info('Wrote the following as snapcraft.yaml.')
    print()
    print(yaml)
    sys.exit(0)


def shell(args):
    config = snapcraft.yaml.Config()
    common.env = config.stage_env()
    userCommand = args.userCommand
    if not userCommand:
        userCommand = ['/usr/bin/env', 'PS1=\[\e[1;32m\]snapcraft:\w\$\[\e[0m\] ', '/bin/bash', '--norc']
    common.run(userCommand)


def wrap_exe(relexepath):
    snapdir = common.get_snapdir()
    exepath = os.path.join(snapdir, relexepath)
    wrappath = exepath + '.wrapper'

    try:
        os.remove(wrappath)
    except Exception:
        pass

    wrapexec = '$SNAP_APP_PATH/{}'.format(relexepath)
    if not os.path.exists(exepath) and '/' not in relexepath:
        # If it doesn't exist it might be in the path
        logger.info('Checking to see if "{}" is in the $PATH'.format(relexepath))
        with tempfile.NamedTemporaryFile() as tempf:
            script = ('#!/bin/sh\n' +
                      '{}\n'.format(snapcraft.common.assemble_env()) +
                      'which "{}"'.format(relexepath))
            tempf.write(bytes(script, 'UTF-8'))
            if snapcraft.common.run(['/bin/sh', tempf.name], cwd=snapdir):
                wrapexec = relexepath
            else:
                logger.warning('Warning: unable to find "{}" in the path'.format(relexepath))

    assembled_env = common.assemble_env().replace(snapdir, '$SNAP_APP_PATH')
    script = ('#!/bin/sh\n' +
              '{}\n'.format(assembled_env) +
              'exec "{}" $*'.format(wrapexec))

    with open(wrappath, 'w+') as f:
        f.write(script)

    os.chmod(wrappath, 0o755)

    return os.path.relpath(wrappath, snapdir)


def snap(args):
    cmd(args)

    # Ensure the snappy metadata files are correct
    config = snapcraft.yaml.Config()

    if 'snappy-metadata' in config.data:
        common.run(
            ['cp', '-arvT', config.data['snappy-metadata'], common.get_snapdir() + '/meta'])
    if not os.path.exists('snap/meta/package.yaml'):
        logger.error("Missing snappy metadata file 'meta/package.yaml'.  Try specifying 'snappy-metadata' in snapcraft.yaml, pointing to a meta directory in your source tree.")
        sys.exit(1)

    # wrap all included commands
    with open("snap/meta/package.yaml", 'r') as f:
        package = yaml.load(f)

    common.env = config.snap_env()

    def replace_cmd(execparts, cmd):
        newparts = [cmd] + execparts[1:]
        return ' '.join([shlex.quote(x) for x in newparts])

    for binary in package.get('binaries', []):
        execparts = shlex.split(binary.get('exec', binary['name']))
        execwrap = wrap_exe(execparts[0])
        if 'exec' in binary:
            binary['exec'] = replace_cmd(execparts, execwrap)
        else:
            binary['name'] = os.path.basename(binary['name'])
            binary['exec'] = replace_cmd(execparts, execwrap)

    for binary in package.get('services', []):
        startpath = binary.get('start')
        if startpath:
            startparts = shlex.split(startpath)
            startwrap = wrap_exe(startparts[0])
            binary['start'] = replace_cmd(startparts, startwrap)
        stoppath = binary.get('stop')
        if stoppath:
            stopparts = shlex.split(stoppath)
            stopwrap = wrap_exe(stopparts[0])
            binary['stop'] = replace_cmd(stopparts, stopwrap)

    with open("snap/meta/package.yaml", 'w') as f:
        yaml.dump(package, f, default_flow_style=False)


def assemble(args):
    args.cmd = 'snap'
    snap(args)
    common.run(['snappy', 'build', common.get_snapdir()])


def run(args):
    qemudir = os.path.join(os.getcwd(), "image")
    qemu_img = os.path.join(qemudir, "15.04.img")
    if not os.path.exists(qemu_img):
            try:
                os.makedirs(qemudir)
            except FileExistsError:
                pass
            common.run(
                ['sudo', 'ubuntu-device-flash', 'core', '--developer-mode', '--enable-ssh', '15.04', '-o', qemu_img],
                cwd=qemudir)
    qemu = subprocess.Popen(
        ["kvm", "-m", "768", "-nographic",
         "-snapshot", "-redir", "tcp:8022::22", qemu_img],
        stdin=subprocess.PIPE)
    n = tempfile.NamedTemporaryFile()
    ssh_opts = [
        "-oStrictHostKeyChecking=no",
        "-oUserKnownHostsFile=%s" % n.name
    ]
    while True:
        ret_code = subprocess.call(
            ["ssh"] + ssh_opts +
            ["ubuntu@localhost", "-p", "8022", "true"])
        if ret_code == 0:
            break
        print("Waiting for device")
        time.sleep(1)
    snap_dir = os.path.join(os.getcwd(), "snap")
    # copy the snap
    snaps = glob.glob(snap_dir + "/*.snap")
    subprocess.call(
        ["scp"] + ssh_opts + [
            "-P", "8022", "-r"] + snaps + ["ubuntu@localhost:~/"])
    # install the snap
    ret_code = subprocess.call(
        ["ssh"] + ssh_opts +
        ["ubuntu@localhost", "-p", "8022", "sudo snappy install  *.snap"])
    # "login"
    subprocess.call(
        ["ssh"] + ssh_opts + ["-p", "8022", "ubuntu@localhost"],
        preexec_fn=os.setsid)
    qemu.kill()


def check_for_collisions(parts):
    partsFiles = {}
    for part in parts:
        # Gather our own files up
        partFiles = set()
        for root, dirs, files in os.walk(part.installdir):
            partFiles |= set([os.path.join(root, f) for f in files])
        partFiles = set([os.path.relpath(x, part.installdir) for x in partFiles])

        # Scan previous parts for collisions
        for otherPartName in partsFiles:
            common = partFiles & partsFiles[otherPartName]
            if common:
                logger.error('Error: parts %s and %s have the following files in common:\n  %s' % (otherPartName, part.names()[0], '\n  '.join(sorted(common))))
                return False

        # And add our files to the list
        partsFiles[part.names()[0]] = partFiles

    return True


def cmd(args):
    forceAll = args.force
    forceCommand = None

    cmds = [args.cmd]

    if cmds[0] in common.COMMAND_ORDER:
        forceCommand = cmds[0]
        cmds = common.COMMAND_ORDER[0:common.COMMAND_ORDER.index(cmds[0]) + 1]

    config = snapcraft.yaml.Config()

    # Install local packages that we need
    if config.build_tools:
        newPackages = []
        for checkpkg in config.build_tools:
            if subprocess.call(['dpkg-query', '-s', checkpkg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
                newPackages.append(checkpkg)
        if newPackages:
            print("Installing required packages on the host system: " + ", ".join(newPackages))
            subprocess.call(['sudo', 'apt-get', '-y', 'install'] + newPackages, stdout=subprocess.DEVNULL)

    for part in config.all_parts:
        for cmd in cmds:
            if cmd == 'stage':
                # This ends up running multiple times, as each part gets to its
                # staging cmd.  That's inefficient, but largely OK.
                # FIXME: fix the above by iterating over cmds before iterating
                # all_parts.  But then we need to make sure we continue to handle
                # cases like go, where you want go built before trying to pull
                # a go project.
                if not check_for_collisions(config.all_parts):
                    sys.exit(1)

            common.env = config.build_env_for_part(part)
            force = forceAll or cmd == forceCommand
            if not getattr(part, cmd)(force=force):
                logger.error('Failed doing %s for %s!' % (cmd, part.names()[0]))
                sys.exit(1)
