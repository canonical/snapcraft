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
import os
import snapcraft.common
import snapcraft.plugin
import snapcraft.yaml
import subprocess
import sys
import tempfile
import time


def init(args):
    if os.path.exists("snapcraft.yaml"):
        snapcraft.common.log("snapcraft.yaml already exists!", file=sys.stderr)
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
    snapcraft.common.log("Wrote the following as snapcraft.yaml.")
    print()
    print(yaml)
    sys.exit(0)


def shell(args):
    config = snapcraft.yaml.Config()
    snapcraft.common.env = config.stage_env()
    userCommand = args.userCommand
    if not userCommand:
        userCommand = ['/usr/bin/env', 'PS1=\[\e[1;32m\]snapcraft:\w\$\[\e[0m\] ', '/bin/bash', '--norc']
    snapcraft.common.run(userCommand)


def assemble(args):
    args.cmd = 'snap'
    cmd(args)

    config = snapcraft.yaml.Config()

    snapcraft.common.run(
        ['cp', '-arv', config.data["snap"]["meta"], snapcraft.common.snapdir])

    # wrap all included commands
    snapcraft.common.env = config.snap_env()
    script = "#!/bin/sh\n%s\nexec %%s $*" % snapcraft.common.assemble_env().replace(snapcraft.common.snapdir, "$SNAP_APP_PATH")

    def wrap_bins(bindir):
        absbindir = os.path.join(snapcraft.common.snapdir, bindir)
        if not os.path.exists(absbindir):
            return
        for exe in os.listdir(absbindir):
            if exe.endswith('.real'):
                continue
            exePath = os.path.join(absbindir, exe)
            try:
                os.remove(exePath + '.real')
            except:
                pass
            os.rename(exePath, exePath + '.real')
            with open(exePath, 'w+') as f:
                f.write(script % ('"$SNAP_APP_PATH/' + bindir + '/' + exe + '.real"'))
            os.chmod(exePath, 0o755)
    wrap_bins('bin')
    wrap_bins('usr/bin')

    snapcraft.common.run(['snappy', 'build', snapcraft.common.snapdir])


def run(args):
    qemudir = os.path.join(os.getcwd(), "image")
    qemu_img = os.path.join(qemudir, "15.04.img")
    if not os.path.exists(qemu_img):
            try:
                os.makedirs(qemudir)
            except FileExistsError:
                pass
            snapcraft.common.run(
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
                snapcraft.common.log("Error: parts %s and %s have the following files in common:\n  %s" % (otherPartName, part.names()[0], '\n  '.join(sorted(common))))
                return False

        # And add our files to the list
        partsFiles[part.names()[0]] = partFiles

    return True


def cmd(args):
    forceAll = args.force
    forceCommand = None

    if args.cmd == "all":
        cmds = ['snap']
    else:
        cmds = [args.cmd]

    if cmds[0] in snapcraft.common.commandOrder:
        forceCommand = cmds[0]
        cmds = snapcraft.common.commandOrder[0:snapcraft.common.commandOrder.index(cmds[0]) + 1]

    config = snapcraft.yaml.Config()

    # Install local packages that we need
    if config.systemPackages:
        newPackages = []
        for checkpkg in config.systemPackages:
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

            snapcraft.common.env = config.build_env_for_part(part)
            force = forceAll or cmd == forceCommand
            if not getattr(part, cmd)(force=force):
                snapcraft.common.log("Failed doing %s for %s!" % (cmd, part.names()[0]))
                sys.exit(1)
