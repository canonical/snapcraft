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


import logging
import os
import subprocess
import time
from typing import List

from ._containerbuild import Containerbuild

from snapcraft.internal.errors import (
        ContainerConnectionError,
        ContainerRunError,
        SnapcraftEnvironmentError,
)
from snapcraft.internal import lifecycle
from snapcraft.cli import echo

logger = logging.getLogger(__name__)


class Project(Containerbuild):

    def __init__(self, *, output, source, project_options,
                 metadata, remote=None):
        super().__init__(output=output, source=source,
                         project_options=project_options,
                         metadata=metadata, container_name=metadata['name'],
                         remote=remote)
        self._processes = []

    def _ensure_container(self):
        new_container = not self._get_container_status()
        if new_container:
            try:
                subprocess.check_call([
                    'lxc', 'init', self._image, self._container_name])
            except subprocess.CalledProcessError as e:
                raise ContainerConnectionError('Failed to setup container')
        if self._get_container_status()['status'] == 'Stopped':
            self._configure_container()
            try:
                subprocess.check_call([
                    'lxc', 'start', self._container_name])
            except subprocess.CalledProcessError:
                msg = 'The container could not be started.'
                if self._remote == 'local':
                    msg += ('\nThe files /etc/subuid and /etc/subgid need to '
                            'contain this line for mounting the local folder:'
                            '\n    root:1000:1'
                            '\nNote: Add the line to both files, do not '
                            'remove any existing lines.'
                            '\nRestart lxd after making this change.')
                raise ContainerConnectionError(msg)
        self._wait_for_network()
        if new_container:
            self._container_run(['apt-get', 'update'])
            # Because of https://bugs.launchpad.net/snappy/+bug/1628289
            # Needed to run snapcraft as a snap and build-snaps
            self._container_run(['apt-get', 'install', 'squashfuse', '-y'])
        self._inject_snapcraft(new_container=new_container)

    def _configure_container(self):
        super()._configure_container()
        if self._remote == 'local':
            # Map host user to root (0) inside container
            subprocess.check_call([
                'lxc', 'config', 'set', self._container_name,
                'raw.idmap',
                'both {} {}'.format(os.getenv('SUDO_UID', os.getuid()),
                                    0)])
        # Remove existing device (to ensure we update old containers)
        devices = self._get_container_status()['devices']
        if self._project_folder in devices:
            subprocess.check_call([
                'lxc', 'config', 'device', 'remove', self._container_name,
                self._project_folder])
        if 'fuse' not in devices:
            subprocess.check_call([
                'lxc', 'config', 'device', 'add', self._container_name,
                'fuse', 'unix-char', 'path=/dev/fuse'
                ])

    def _setup_project(self):
        if self._remote != 'local':
            self._setup_user()
            logger.info('Mounting {} into container'.format(self._source))
            return self._remote_mount(self._project_folder, self._source)

        devices = self._get_container_status()['devices']
        if self._project_folder not in devices:
            logger.info('Mounting {} into container'.format(self._source))
            subprocess.check_call([
                'lxc', 'config', 'device', 'add', self._container_name,
                self._project_folder, 'disk', 'source={}'.format(self._source),
                'path={}'.format(self._project_folder)])

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
        except subprocess.CalledProcessError:
            raise SnapcraftEnvironmentError(
                'sftp-server seems to be installed but could not be run.\n'
                )

        # Use sshfs in slave mode to reverse mount the destination
        self._container_run(['apt-get', 'install', '-y', 'sshfs'])
        self._container_run(['mkdir', '-p', destination], user=self._user)
        self._background_process_run([
            'lxc', 'exec', self._container_name, '--',
            'sudo', '-H', '-u', self._user,
            'sshfs', '-o', 'slave', '-o', 'nonempty',
            ':{}'.format(source), destination],
            stdin=stdin2, stdout=stdout1)

        # It may take a second or two for sshfs to come up
        retry_count = 5
        while retry_count:
            time.sleep(1)
            if subprocess.check_output([
                    'lxc', 'exec', self._container_name, '--',
                    'sudo', '-H', '-u', self._user,
                    'ls', self._project_folder]):
                return
            retry_count -= 1
        raise ContainerConnectionError(
            'The project folder could not be mounted.\n'
            'Fuse must be enabled on the LXD host.\n'
            'You can run the following command to enable it:\n'
            'echo Y | sudo tee /sys/module/fuse/parameters/userns_mounts')

    def _setup_user(self):
        # If we run as root or sudo we don't need a user in the container
        if os.geteuid() == 0:
            return
        # Setup user mirroring host user with sudo access
        self._user = os.environ['USER']
        self._project_folder = '/home/{}/build_{}'.format(
            self._user, self._metadata['name'])
        logger.debug('Setting up user {!r} in container'.format(self._user))
        try:
            self._container_run([
                'useradd', self._user, '--create-home'])
        except ContainerRunError as e:
            # Exit code 9 'username already in use' is safe to ignore
            if e.exit_code != 9:
                raise e
        self._container_run([
            'usermod', self._user, '-o',
            '-u', str(os.getuid()), '-G', 'sudo'])
        self._container_run([
            'chown', '{}:{}'.format(self._user, self._user),
            '/home/{}'.format(self._user)])
        subprocess.check_output([
            'lxc', 'exec', self._container_name, '--',
            'tee', '-a', '/etc/sudoers'],
            input='{} ALL=(ALL) NOPASSWD: ALL\n'.format(self._user).encode())

    def _background_process_run(self, cmd, **kwargs):
        self._processes += [subprocess.Popen(cmd, **kwargs)]

    def _finish(self):
        for process in self._processes:
            logger.debug('Terminating {}'.format(process.args))
            process.terminate()

    def refresh(self):
        with self._container_running():
            self._container_run(['apt-get', 'update'])
            self._container_run(['apt-get', 'upgrade', '-y'])
            self._container_run(['snap', 'refresh'])

    def clean(self, parts: List[str], step: str):
        # clean with no parts deletes the container
        if not step:
            if not parts:
                self._ensure_remote()
                if self._get_container_status():
                    print('Deleting {}'.format(self._container_name))
                    subprocess.check_call([
                        'lxc', 'delete', '-f', self._container_name])
            step = 'pull'
        # clean normally, without involving the container
        if step == 'strip':
            echo.warning('DEPRECATED: Use `prime` instead of `strip` '
                         'as the step to clean')
            step = 'prime'
        lifecycle.clean(self._project_options, parts, step)
