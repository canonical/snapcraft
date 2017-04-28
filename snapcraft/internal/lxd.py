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
import sys
from contextlib import contextmanager
from subprocess import check_call, check_output, CalledProcessError, Popen
from time import sleep

import petname

from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.internal import common

logger = logging.getLogger(__name__)

_NETWORK_PROBE_COMMAND = \
    'import urllib.request; urllib.request.urlopen("{}", timeout=5)'.format(
        'http://start.ubuntu.com/connectivity-check.html')
_PROXY_KEYS = ['http_proxy', 'https_proxy', 'no_proxy', 'ftp_proxy']


class Containerbuild:

    def __init__(self, *, output, source, project_options,
                 metadata, container_name, remote=None):
        if not output:
            output = common.format_snap_name(metadata)
        self._snap_output = output
        self._source = os.path.realpath(source)
        self._project_options = project_options
        self._metadata = metadata
        self._project_folder = 'build_{}'.format(metadata['name'])

        if not remote:
            remote = _get_default_remote()
        _verify_remote(remote)
        self._container_name = '{}:snapcraft-{}'.format(remote, container_name)

    def _push_file(self, src, dst):
        check_call(['lxc', 'file', 'push',
                    src, '{}/{}'.format(self._container_name, dst)])

    def _pull_file(self, src, dst):
        check_call(['lxc', 'file', 'pull',
                    '{}/{}'.format(self._container_name, src), dst])

    def _container_run(self, cmd, stdin=None, stdout=None):
        # Set HOME here as 'lxc config set ... environment.HOME' doesn't work
        # Use 'cd' because --env has no effect with sshfs mounts
        check_call(['lxc', 'exec', self._container_name, '--',
                   'bash', '-c', 'cd /{}; {}'.format(
                       self._project_folder,
                       ' '.join(pipes.quote(arg) for arg in cmd))],
                   stdin=stdin, stdout=stdout)

    def _ensure_container(self):
        check_call([
            'lxc', 'launch', '-e',
            'ubuntu:xenial/{}'.format(self._project_options.deb_arch),
            self._container_name])
        check_call([
            'lxc', 'config', 'set', self._container_name,
            'environment.SNAPCRAFT_SETUP_CORE', '1'])

    @contextmanager
    def _ensure_started(self):
        try:
            self._ensure_container()
            yield
        finally:
            # Stopping takes a while and lxc doesn't print anything.
            print('Stopping {}'.format(self._container_name))
            check_call(['lxc', 'stop', '-f', self._container_name])

    def _install_packages(self, packages):
        try:
            self._wait_for_network()
            self._container_run(['apt-get', 'update'])
            self._container_run(['apt-get', 'install', '-y', *packages])
        finally:
            # Always remove apt lock in case we stop during install
            self._container_run(['rm', '-f', '/var/lib/apt/lists/lock'])

    def execute(self, step='snap', args=None):
        with self._ensure_started():
            self._setup_project()
            self._install_packages(['snapcraft'])
            command = ['snapcraft', step]
            if step == 'snap':
                command += ['--output', self._snap_output]
            if args:
                command += args
            try:
                self._container_run(command)
            except CalledProcessError as e:
                if self._project_options.debug:
                    logger.info('Debug mode enabled, dropping into a shell')
                    self._container_run(['bash', '-i'])
                else:
                    raise e
            else:
                self._finish()

    def _setup_project(self):
        logger.info('Setting up container with project assets')
        tar_filename = self._source
        dst = os.path.join(self._project_folder,
                           os.path.basename(tar_filename))
        self._container_run(['mkdir', self._project_folder])
        self._push_file(tar_filename, dst)
        self._container_run(['tar', 'xvf', os.path.basename(tar_filename)])

    def _finish(self):
        src = os.path.join(self._project_folder, self._snap_output)
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

    def _get_container_status(self):
        containers = json.loads(check_output([
            'lxc', 'list', '--format=json', self._container_name]).decode())
        for container in containers:
            remote, container_name = self._container_name.split(':')
            if container['name'] == container_name:
                return container

    def _ensure_container(self):
        if not self._get_container_status():
            check_call([
                'lxc', 'init',
                'ubuntu:xenial/{}'.format(self._project_options.deb_arch),
                self._container_name])
            check_call([
                'lxc', 'config', 'set', self._container_name,
                'environment.SNAPCRAFT_SETUP_CORE', '1'])
            # Map host user to root inside container
            check_call([
                'lxc', 'config', 'set', self._container_name,
                'raw.idmap', 'both 1000 0'])
        if self._get_container_status()['status'] == 'Stopped':
            check_call([
                'lxc', 'start', self._container_name])

    def _setup_project(self):
        self._ensure_mount(self._project_folder, self._source)

    def _container_login(self):
        network = self._get_container_status()['state']['network']['eth0']
        for address in network['addresses']:
            if address['family'] == 'inet':
                return address['address'], '22'
        raise RuntimeError('No IP found for {}'.format(self._container_name))

    def _background(self, cmd, stdin=None, stdout=None):
        self._processes += [Popen(cmd, stdin=stdin, stdout=stdout)]

    def _ensure_mount(self, destination, source):
        logger.info('Mounting {} into container'.format(source))
        remote, container_name = self._container_name.split(':')
        if remote != 'local':
            # Remove project folder in case it was used "locally" before
            devices = self._get_container_status()['devices']
            if destination in devices:
                check_call([
                    'lxc', 'config', 'device', 'remove', self._container_name,
                    destination, 'disk', 'source={}'.format(source),
                    'path=/{}'.format(destination)])

            self._install_packages(['sshfs'])
            keyfile = 'id_{}'.format(self._container_name)
            if not os.path.exists(keyfile):
                check_output(['ssh-keygen', '-o', '-N', '', '-f', keyfile])
            self._container_run(['mkdir', '-p', '/root/.ssh'])
            self._container_run(['chmod', '700', '/root/.ssh'])
            self._container_run(['tee', '-a',
                                 '/root/.ssh/authorized_keys'],
                                stdin=open('{}.pub'.format(keyfile), 'r'))
            self._container_run(['chmod', '600',
                                 '/root/.ssh/authorized_keys'])
            self._container_run(['mkdir', '-p', '/{}'.format(destination)])
            self._container_run(['mkdir', '-p', source])
            r1, w1 = os.pipe()
            r2, w2 = os.pipe()
            ssh_hostname, ssh_port = self._container_login()
            logger.info('Connecting via SSH to {}:{}'.format(
                ssh_hostname, ssh_port))
            self._background(['/usr/lib/sftp-server'], stdin=r1, stdout=w2)
            self._background(['ssh', '-C', '-F', '/dev/null',
                              '-o', 'IdentityFile={}'.format(keyfile),
                              '-o', 'StrictHostKeyChecking=no',
                              '-o', 'UserKnownHostsFile=/dev/null',
                              '-o', 'User=root',
                              '-p', ssh_port, ssh_hostname,
                              'sshfs -o slave -o nonempty :{} /{}'.format(
                                  source, destination)],
                             stdin=r2, stdout=w1)
            # -o allow_other, -o, idmap=user,uid=1000,gid=1000
        else:
            devices = self._get_container_status()['devices']
            if destination not in devices:
                check_call([
                    'lxc', 'config', 'device', 'add', self._container_name,
                    destination, 'disk', 'source={}'.format(source),
                    'path=/{}'.format(destination)])

    def _finish(self):
        for process in self._processes:
            logger.info('Terminating {}'.format(process.args))
            process.terminate()

    def execute(self, step='snap', args=None):
        super().execute(step, args)
        if step == 'clean' and not args:
            print('Deleting {}'.format(self._container_name))
            check_call(['lxc', 'delete', '-f', self._container_name])


def _get_default_remote():
    """Query and return the default lxd remote.

    Use the lxc command to query for the default lxd remote. In most
    cases this will return the local remote.

    :returns: default lxd remote.
    :rtype: string.
    :raises snapcraft.internal.errors.SnapcraftEnvironmentError:
        raised if the lxc call fails.
    """
    try:
        default_remote = check_output(['lxc', 'remote', 'get-default'])
    except CalledProcessError:
        raise SnapcraftEnvironmentError(
            'You must have LXD installed in order to use cleanbuild. '
            'However, it is either not installed or not configured '
            'properly.\n'
            'Refer to the documentation at '
            'https://linuxcontainers.org/lxd/getting-started-cli.')
    return default_remote.decode(sys.getfilesystemencoding()).strip()


def _verify_remote(remote):
    """Verify that the lxd remote exists.

    :param str remote: the lxd remote to verify.
    :raises snapcraft.internal.errors.SnapcraftEnvironmentError:
        raised if the lxc call listing the remote fails.
    """
    # There is no easy way to grep the results from `lxc remote list`
    # so we try and execute a simple operation against the remote.
    try:
        check_output(['lxc', 'list', '{}:'.format(remote)])
    except CalledProcessError as e:
        raise SnapcraftEnvironmentError(
            'There are either no permissions or the remote {!r} '
            'does not exist.\n'
            'Verify the existing remotes by running `lxc remote list`\n'
            'To setup a new remote, follow the instructions at\n'
            'https://linuxcontainers.org/lxd/getting-started-cli/'
            '#multiple-hosts'.format(remote)) from e
