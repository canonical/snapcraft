#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import json
import logging
import os
import pipes
import shutil
import sys
import tempfile
from contextlib import contextmanager
from subprocess import check_call, check_output, CalledProcessError, Popen
from time import sleep
from urllib import parse
import requests
import requests_unixsocket

import petname
import yaml

from snapcraft.internal.errors import (
        ContainerConnectionError,
        SnapcraftEnvironmentError,
        SnapdError,
)
from snapcraft.internal import common
from snapcraft.internal.repo import snaps
from snapcraft._options import _get_deb_arch

logger = logging.getLogger(__name__)

_NETWORK_PROBE_COMMAND = \
    'import urllib.request; urllib.request.urlopen("{}", timeout=5)'.format(
        'http://start.ubuntu.com/connectivity-check.html')
_PROXY_KEYS = ['http_proxy', 'https_proxy', 'no_proxy', 'ftp_proxy']
# Canonical store account key
_STORE_KEY = (
    'BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul')


class Containerbuild:

    def __init__(self, *, output, source, project_options,
                 metadata, container_name, remote=None):
        if not output:
            output = common.format_snap_name(metadata)
        self._snap_output = output
        self._source = os.path.realpath(source)
        self._project_options = project_options
        self._metadata = metadata
        self._project_folder = '/root/build_{}'.format(metadata['name'])

        if not remote:
            remote = _get_default_remote()
        _verify_remote(remote)
        self._container_name = '{}:snapcraft-{}'.format(remote, container_name)
        server_environment = self._get_remote_info()['environment']
        # Use the server architecture to avoid emulation overhead
        try:
            kernel = server_environment['kernel_architecture']
        except KeyError:
            kernel = server_environment['kernelarchitecture']
        deb_arch = _get_deb_arch(kernel)
        if not deb_arch:
            raise ContainerConnectionError(
                'Unrecognized server architecture {}'.format(kernel))
        self._image = 'ubuntu:xenial/{}'.format(deb_arch)
        # Use a temporary folder the 'lxd' snap can access
        lxd_common_dir = os.path.expanduser(
            os.path.join('~', 'snap', 'lxd', 'common'))
        os.makedirs(lxd_common_dir, exist_ok=True)
        self.tmp_dir = tempfile.mkdtemp(prefix='snapcraft', dir=lxd_common_dir)

    def _get_remote_info(self):
        remote = self._container_name.split(':')[0]
        return yaml.load(check_output([
            'lxc', 'info', '{}:'.format(remote)]).decode())

    def _push_file(self, src, dst):
        check_call(['lxc', 'file', 'push',
                    src, '{}{}'.format(self._container_name, dst)])

    def _pull_file(self, src, dst):
        check_call(['lxc', 'file', 'pull',
                    '{}{}'.format(self._container_name, src), dst])

    def _container_run(self, cmd, cwd=None, **kwargs):
        sh = ''
        # Automatically wait on lock files before running commands
        if cmd[0] == 'apt-get':
            lock_file = '/var/lib/dpkg/lock'
            if cmd[1] == 'update':
                lock_file = '/var/lib/apt/lists/lock'
            sh += 'while fuser {} >/dev/null 2>&1; do sleep 1; done; '.format(
                lock_file)
        if cwd:
            sh += 'cd {}; '.format(cwd)
        if sh:
            cmd = ['sh', '-c', '{}{}'.format(sh,
                   ' '.join(pipes.quote(arg) for arg in cmd))]
        check_call(['lxc', 'exec', self._container_name, '--'] + cmd,
                   **kwargs)

    def _ensure_container(self):
        check_call([
            'lxc', 'launch', '-e', self._image, self._container_name])
        check_call([
            'lxc', 'config', 'set', self._container_name,
            'environment.SNAPCRAFT_SETUP_CORE', '1'])
        # Necessary to read asset files with non-ascii characters.
        check_call([
            'lxc', 'config', 'set', self._container_name,
            'environment.LC_ALL', 'C.UTF-8'])

    @contextmanager
    def _ensure_started(self):
        try:
            self._ensure_container()
            yield
        finally:
            status = self._get_container_status()
            if status and status['status'] == 'Running':
                # Stopping takes a while and lxc doesn't print anything.
                print('Stopping {}'.format(self._container_name))
                check_call(['lxc', 'stop', '-f', self._container_name])

    def _get_container_status(self):
        containers = json.loads(check_output([
            'lxc', 'list', '--format=json', self._container_name]).decode())
        for container in containers:
            if container['name'] == self._container_name.split(':')[-1]:
                return container

    def execute(self, step='snap', args=None):
        with self._ensure_started():
            self._wait_for_network()
            self._container_run(['apt-get', 'update'])
            self._setup_project()
            self._inject_snapcraft()
            command = ['snapcraft', step]
            if step == 'snap':
                command += ['--output', self._snap_output]
            # Pass on target arch if specified
            # If not specified it defaults to the LXD architecture
            if self._project_options.target_arch:
                command += ['--target-arch', self._project_options.target_arch]
            if args:
                command += args
            try:
                self._container_run(command, cwd=self._project_folder)
            except CalledProcessError as e:
                if self._project_options.debug:
                    logger.info('Debug mode enabled, dropping into a shell')
                    self._container_run(['bash', '-i'])
                else:
                    raise e
            else:
                # Remove temporary folder if everything went well
                shutil.rmtree(self.tmp_dir)
                self._finish()

    def _setup_project(self):
        logger.info('Setting up container with project assets')
        tar_filename = self._source
        # os.sep needs to be `/` and on Windows it will be set to `\`
        dst = '{}/{}'.format(self._project_folder,
                             os.path.basename(tar_filename))
        self._container_run(['mkdir', self._project_folder])
        self._push_file(tar_filename, dst)
        self._container_run(['tar', 'xvf', os.path.basename(tar_filename)],
                            cwd=self._project_folder)

    def _inject_snapcraft(self):
        if common.is_snap():
            # Because of https://bugs.launchpad.net/snappy/+bug/1628289
            self._container_run(['apt-get', 'install', 'squashfuse', '-y'])

            # Push core snap into container
            self._inject_snap('core')
            self._inject_snap('snapcraft')
        else:
            self._container_run(['apt-get', 'install', 'snapcraft', '-y'])

    def _inject_snap(self, name):
        session = requests_unixsocket.Session()
        # Cf. https://github.com/snapcore/snapd/wiki/REST-API#get-v2snapsname
        # TODO use get_local_snap info from the snaps module.
        slug = 'snaps/{}'.format(parse.quote(name, safe=''))
        api = snaps.get_snapd_socket_path_template().format(slug)
        try:
            json = session.request('GET', api).json()
        except requests.exceptions.ConnectionError as e:
            raise SnapdError(
                'Error connecting to {}'.format(api)) from e
        if json['type'] == 'error':
            raise SnapdError(
                'Error querying {!r} snap: {}'.format(
                    name, json['result']['message']))
        id = json['result']['id']
        # Lookup confinement to know if we need to --classic when installing
        is_classic = json['result']['confinement'] == 'classic'
        # Revisions are unique, so we don't need to know the channel
        rev = json['result']['revision']

        if not rev.startswith('x'):
            self._inject_assertions('{}_{}.assert'.format(name, rev), [
                ['account-key', 'public-key-sha3-384={}'.format(_STORE_KEY)],
                ['snap-declaration', 'snap-name={}'.format(name)],
                ['snap-revision', 'snap-revision={}'.format(rev),
                 'snap-id={}'.format(id)],
            ])

        # https://github.com/snapcore/snapd/blob/master/snap/info.go
        # MountFile
        filename = '{}_{}.snap'.format(name, rev)
        # https://github.com/snapcore/snapd/blob/master/dirs/dirs.go
        # CoreLibExecDir
        installed = os.path.join(os.path.sep, 'var', 'lib', 'snapd', 'snaps',
                                 filename)

        filepath = os.path.join(self.tmp_dir, filename)
        if rev.startswith('x'):
            logger.info('Making {} user-accessible'.format(filename))
            check_call(['sudo', 'cp', installed, filepath])
            check_call(['sudo', 'chown', str(os.getuid()), filepath])
        else:
            shutil.copyfile(installed, filepath)
        container_filename = os.path.join(os.sep, 'run', filename)
        self._push_file(filepath, container_filename)
        logger.info('Installing {}'.format(container_filename))
        cmd = ['snap', 'install', container_filename]
        if rev.startswith('x'):
            cmd.append('--dangerous')
        if is_classic:
            cmd.append('--classic')
        self._container_run(cmd)

    def _inject_assertions(self, filename, assertions):
        filepath = os.path.join(self.tmp_dir, filename)
        with open(filepath, 'wb') as f:
            for assertion in assertions:
                logger.info('Looking up assertion {}'.format(assertion))
                f.write(check_output(['snap', 'known', *assertion]))
                f.write(b'\n')
        container_filename = os.path.join(os.path.sep, 'run', filename)
        self._push_file(filepath, container_filename)
        logger.info('Adding assertion {}'.format(filename))
        self._container_run(['snap', 'ack', container_filename])

    def _finish(self):
        # os.sep needs to be `/` and on Windows it will be set to `\`
        src = '{}/{}'.format(self._project_folder, self._snap_output)
        self._pull_file(src, self._snap_output)
        logger.info('Retrieved {}'.format(self._snap_output))

    def _wait_for_network(self):
        logger.info('Waiting for a network connection...')
        not_connected = True
        retry_count = 5
        while not_connected:
            sleep(5)
            try:
                self._container_run(['python3', '-c', _NETWORK_PROBE_COMMAND])
                not_connected = False
            except CalledProcessError as e:
                retry_count -= 1
                if retry_count == 0:
                    raise e
        logger.info('Network connection established')


class Cleanbuilder(Containerbuild):

    def __init__(self, *, output=None, source, project_options,
                 metadata=None, remote=None):
        container_name = petname.Generate(3, '-')
        super().__init__(output=output, source=source,
                         project_options=project_options, metadata=metadata,
                         container_name=container_name, remote=remote)


class Project(Containerbuild):

    def __init__(self, *, output, source, project_options,
                 metadata, remote=None):
        super().__init__(output=output, source=source,
                         project_options=project_options,
                         metadata=metadata, container_name=metadata['name'],
                         remote=remote)
        self._processes = []

    def _ensure_container(self):
        if not self._get_container_status():
            check_call([
                'lxc', 'init', self._image, self._container_name])
        if self._get_container_status()['status'] == 'Stopped':
            check_call([
                'lxc', 'config', 'set', self._container_name,
                'environment.SNAPCRAFT_SETUP_CORE', '1'])
            # Necessary to read asset files with non-ascii characters.
            check_call([
                'lxc', 'config', 'set', self._container_name,
                'environment.LC_ALL', 'C.UTF-8'])
            if self._container_name.startswith('local:'):
                # Map host user to root inside container
                check_call([
                    'lxc', 'config', 'set', self._container_name,
                    'raw.idmap', 'both {} 0'.format(os.getuid())])
            # Remove existing device (to ensure we update old containers)
            devices = self._get_container_status()['devices']
            if self._project_folder in devices:
                check_call([
                    'lxc', 'config', 'device', 'remove', self._container_name,
                    self._project_folder])
            if 'fuse' not in devices:
                check_call([
                    'lxc', 'config', 'device', 'add', self._container_name,
                    'fuse', 'unix-char', 'path=/dev/fuse'
                    ])
            try:
                check_call([
                    'lxc', 'start', self._container_name])
            except CalledProcessError:
                msg = 'The container could not be started.'
                if self._container_name.startswith('local:'):
                    msg += ('\nThe files /etc/subuid and /etc/subgid need to '
                            'contain this line for mounting the local folder:'
                            '\n    root:1000:1'
                            '\nNote: Add the line to both files, do not '
                            'remove any existing lines.'
                            '\nRestart lxd after making this change.')
                raise ContainerConnectionError(msg)

    def _setup_project(self):
        self._ensure_mount(self._project_folder, self._source)

    def _ensure_mount(self, destination, source):
        logger.info('Mounting {} into container'.format(source))
        if not self._container_name.startswith('local:'):
            return self._remote_mount(destination, source)

        devices = self._get_container_status()['devices']
        if destination not in devices:
            check_call([
                'lxc', 'config', 'device', 'add', self._container_name,
                destination, 'disk', 'source={}'.format(source),
                'path={}'.format(destination)])

    def _remote_mount(self, destination, source):
        # Pipes for sshfs and sftp-server to communicate
        stdin1, stdout1 = os.pipe()
        stdin2, stdout2 = os.pipe()
        # XXX: This needs to be extended once we support other distros
        try:
            self._background_process_run(['/usr/lib/sftp-server'],
                                         stdin=stdin1, stdout=stdout2)
        except FileNotFoundError:
            raise SnapcraftEnvironmentError(
                'You must have openssh-sftp-server installed to use a LXD '
                'remote on a different host.\n'
                )
        except CalledProcessError:
            raise SnapcraftEnvironmentError(
                'sftp-server seems to be installed but could not be run.\n'
                )

        # Use sshfs in slave mode to reverse mount the destination
        self._container_run(['apt-get', 'install', '-y', 'sshfs'])
        self._container_run(['mkdir', '-p', destination])
        self._background_process_run([
            'lxc', 'exec', self._container_name, '--',
            'sshfs', '-o', 'slave', '-o', 'nonempty',
            ':{}'.format(source), destination],
            stdin=stdin2, stdout=stdout1)

        # It may take a second or two for sshfs to come up
        retry_count = 5
        while retry_count:
            sleep(1)
            if check_output(['lxc', 'exec', self._container_name, '--',
                             'ls', self._project_folder]):
                return
            retry_count -= 1
        raise ContainerConnectionError(
            'The project folder could not be mounted.\n'
            'Fuse must be enabled on the LXD host.\n'
            'You can run the following command to enable it:\n'
            'echo Y | sudo tee /sys/module/fuse/parameters/userns_mounts')

    def _background_process_run(self, cmd, **kwargs):
        self._processes += [Popen(cmd, **kwargs)]

    def _finish(self):
        for process in self._processes:
            logger.info('Terminating {}'.format(process.args))
            process.terminate()

    def execute(self, step='snap', args=None):
        # clean with no parts deletes the container
        if step == 'clean' and args == ['--step', 'pull']:
            if self._get_container_status():
                print('Deleting {}'.format(self._container_name))
                check_call(['lxc', 'delete', '-f', self._container_name])
        else:
            super().execute(step, args)


def _get_default_remote():
    """Query and return the default lxd remote.

    Use the lxc command to query for the default lxd remote. In most
    cases this will return the local remote.

    :returns: default lxd remote.
    :rtype: string.
    :raises snapcraft.internal.errors.ContainerConnectionError:
        raised if the lxc call fails.
    """
    try:
        default_remote = check_output(['lxc', 'remote', 'get-default'])
    except FileNotFoundError:
        raise ContainerConnectionError(
            'You must have LXD installed in order to use cleanbuild.')
    except CalledProcessError:
        raise ContainerConnectionError(
            'Something seems to be wrong with your installation of LXD.')
    return default_remote.decode(sys.getfilesystemencoding()).strip()


def _remote_is_valid(remote):
    """Verify that the given string is a valid remote name

    :param str remote: the LXD remote to verify.
    """
    # No colon because it separates remote from container name
    # No slash because it's used for images
    # No spaces
    return not (':' in remote or ' ' in remote or '/' in remote)


def _verify_remote(remote):
    """Verify that the lxd remote exists.

    :param str remote: the lxd remote to verify.
    :raises snapcraft.internal.errors.ContainerConnectionError:
        raised if the lxc call listing the remote fails.
    """
    # There is no easy way to grep the results from `lxc remote list`
    # so we try and execute a simple operation against the remote.
    try:
        check_output(['lxc', 'list', '{}:'.format(remote)])
    except FileNotFoundError:
        raise ContainerConnectionError(
            'You must have LXD installed in order to use cleanbuild.')
    except CalledProcessError as e:
        raise ContainerConnectionError(
            'There are either no permissions or the remote {!r} '
            'does not exist.\n'
            'Verify the existing remotes by running `lxc remote list`\n'
            .format(remote)) from e
