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
import glob
import logging
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import time

import snapcraft.yaml
from snapcraft import common
from snapcraft import lifecycle
from snapcraft import meta

logger = logging.getLogger(__name__)


_TEMPLATE_YAML = r'''name: # the name of the snap
version: # the version of the snap
# The vendor for the snap (replace 'Vendor <email@example.com>')
vendor: Vendor <email@example.com>
summary: # 79 char long summary
description: # A longer description for the snap
icon: # A path to an icon for the package
'''


_config = None


def init(args):
    if os.path.exists("snapcraft.yaml"):
        logger.error('snapcraft.yaml already exists!')
        sys.exit(1)
    yaml = _TEMPLATE_YAML
    if args.part:
        yaml += 'parts:\n'
    for part_name in args.part:
        part = lifecycle.load_plugin(part_name, part_name)
        yaml += '    ' + part.name + ':\n'
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
    config = _load_config()
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

    # This check is to support manual assembly.
    if not os.path.exists(os.path.join(common.get_snapdir(), "meta")):
        arches = [snapcraft.common.get_arch(), ]

        config = _load_config()

        # FIXME this should be done in a more contained manner
        common.env = config.snap_env()

        meta.create(config.data, arches)


def assemble(args):
    args.cmd = 'snap'
    # With all the data in snapcraft.yaml, maybe it's not a good idea to call
    # snap(args) and just do a snappy build if assemble was explicitly called.
    snap(args)
    common.run(['snappy', 'build', common.get_snapdir()])


def _find_latest_private_key():
    """
    Find the latest private key in ~/.ssh.

    :returns:
        Path of the most-recently-modified private SSH key
    :raises LookupError:
        If no such key was found.

    This function tries to mimic the logic found in ``ubuntu-device-flash``. It
    will look for the most recently modified private key in the users' SSH
    configuration directory.
    """
    candidates = []
    ssh_dir = os.path.expanduser('~/.ssh/')
    for filename in os.listdir(ssh_dir):
        # Skip public keys, we want the private key
        if filename.endswith('.pub'):
            continue
        ssh_key = os.path.join(ssh_dir, filename)
        # Skip non-files
        if not os.path.isfile(ssh_key):
            continue
        # Ensure that it is a real ssh key
        with open(ssh_key, 'rb') as stream:
            if stream.readline() != b'-----BEGIN RSA PRIVATE KEY-----\n':
                continue
        candidates.append(ssh_key)
    # Sort the keys by modification time, pick the most recent key
    candidates.sort(key=lambda f: os.stat(f).st_mtime, reverse=True)
    logger.debug("Available ssh public keys: %r", candidates)
    if not candidates:
        raise LookupError("Unable to find any private ssh key")
    return candidates[0]


def run(args):
    # We are mostly making sure we are operating from the correct location. In
    # the future this could do more by using target attribute in snapcraft.yaml
    # to create the correct target image.
    _load_config()
    # Find the ssh key that ubuntu-device-flash would use so that we can use it
    # ourselves as well. This may not be the default key that the user has
    # configured.
    # See: https://bugs.launchpad.net/snapcraft/+bug/1486659
    try:
        ssh_key = _find_latest_private_key()
    except LookupError:
        logger.error("You need to have an SSH key to use this command")
        logger.error("Please generate one with ssh-keygen(1)")
        return 1
    else:
        logger.info("Using the following ssh key: %s", ssh_key)

    # Find available *.snap files to copy into the test VM
    snap_dir = os.path.join(os.getcwd())
    # copy the snap with the largest version number into the test VM
    snaps = glob.glob(snap_dir + "/*.snap")
    snaps.sort()
    if not snaps:
        logger.error("There are no .snap files ready")
        logger.error("Perhaps you forgot to run 'snapcraft assemble'")
        return 1

    qemudir = os.path.join(os.getcwd(), "image")
    qemu_img = os.path.join(qemudir, "15.04.img")
    if not os.path.exists(qemu_img):
        os.makedirs(qemudir, exist_ok=True)
        logger.info(
            'Setting up virtual snappy environment, root access required')
        common.run([
            'sudo', 'ubuntu-device-flash', 'core', '15.04', '--developer-mode',
            '--enable-ssh', '-o', os.path.relpath(qemu_img, qemudir)],
            cwd=qemudir)
    qemu = None
    try:
        # Allow the developer to provide additional arguments to qemu.  This
        # can be used, for example, to pass through USB devices from the host.
        # This can enable a lot of hardware-specific use cases directly inside
        # the snapcraft run workflow.
        #
        # For example:
        # $ export SNAPCRAFT_RUN_QEMU_ARGS=\
        #       "-usb -device usb-host,hostbus=1,hostaddr=10"
        # $ snapcraft run
        qemu_args = os.getenv("SNAPCRAFT_RUN_QEMU_ARGS")
        if qemu_args is not None:
            qemu_args = shlex.split(qemu_args)
        else:
            qemu_args = []
        qemu = subprocess.Popen(
            ["kvm", "-m", "768", "-nographic", "-snapshot", "-redir",
             "tcp:8022::22", qemu_img] + qemu_args, stdin=subprocess.PIPE)
        n = tempfile.NamedTemporaryFile()
        ssh_opts = [
            # We want to login with the specified ssh identity (key)
            '-i', ssh_key,
            # We don't want strict host checking because it's a new VM with a
            # random key each time.
            "-oStrictHostKeyChecking=no",
            # We don't want to pollute the known_hosts file with new entries
            # all the time so let's use a temporary file for that
            "-oUserKnownHostsFile=%s" % n.name,
            # Don't try keyboard interactive authentication, we're expecting to
            # login via the key and if that doesn't work then everything else
            # will fail anyway.
            "-oKbdInteractiveAuthentication=no",
        ]
        while True:
            ret_code = _call(
                ["ssh"] + ssh_opts +
                ["ubuntu@localhost", "-p", "8022", "true"])
            if ret_code == 0:
                break
            print("Waiting for device")
            time.sleep(1)
        # copy the most recent snap into the test VM
        _check_call(
            ["scp"] + ssh_opts + [
                "-P", "8022", snaps[-1], "ubuntu@localhost:~/"])
        # install the snap
        _check_call(
            ["ssh"] + ssh_opts +
            ["ubuntu@localhost", "-p", "8022", "sudo snappy install  *.snap"])
        # "login"
        _check_call(
            ["ssh"] + ssh_opts + ["-p", "8022", "ubuntu@localhost"],
            preexec_fn=os.setsid)
    finally:
        if qemu:
            qemu.kill()


def list_plugins(args=None):
    import pkgutil
    import snapcraft.plugins

    for importer, modname, is_package in pkgutil.iter_modules(
            snapcraft.plugins.__path__):
        if not is_package:
            print(modname.replace('_', '-'))


def clean(args):
    config = _load_config()

    for part in config.all_parts:
        logger.info('Cleaning up for part %r', part.name)
        if os.path.exists(part.partdir):
            shutil.rmtree(part.partdir)

    # parts dir does not contain only generated code.
    if (os.path.exists(common.get_partsdir()) and
            not os.listdir(common.get_partsdir())):
        os.rmdir(common.get_partsdir())

    logger.info('Cleaning up staging area')
    if os.path.exists(common.get_stagedir()):
        shutil.rmtree(common.get_stagedir())

    logger.info('Cleaning up snapping area')
    if os.path.exists(common.get_snapdir()):
        shutil.rmtree(common.get_snapdir())


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

    config = _load_config()
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
                     '-y', 'install'] + new_packages)


def _load_config():
    global _config
    if _config:
        return _config

    try:
        _config = snapcraft.yaml.Config()
        return _config
    except snapcraft.yaml.SnapcraftYamlFileError as e:
        logger.error(
            'Could not find {}.  Are you sure you are in the right '
            'directory?\nTo start a new project, use \'snapcraft '
            'init\''.format(e.file))
        sys.exit(1)
    except snapcraft.yaml.SnapcraftSchemaError as e:
        msg = "Issues while validating snapcraft.yaml: {}".format(e.message)
        logger.error(msg)
        sys.exit(1)
    except snapcraft.yaml.PluginNotDefinedError as e:
        logger.error(
            'Issues while validating snapcraft.yaml: the "plugin" keyword is '
            'missing for the "{}" part.'.format(e.part))
        sys.exit(1)
    except snapcraft.yaml.SnapcraftLogicError as e:
        logger.error('Issue detected while analyzing '
                     'snapcraft.yaml: {}'.format(e.message))
        sys.exit(1)
    except lifecycle.PluginError as e:
        logger.error('Issue while loading plugin: {}'.format(e))
