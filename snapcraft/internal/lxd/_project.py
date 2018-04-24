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
from typing import List

from ._containerbuild import Containerbuild
from snapcraft.internal import errors, lifecycle
from snapcraft.cli import echo

logger = logging.getLogger(__name__)


class Project(Containerbuild):

    def __init__(self, *, output, source, project_options,
                 metadata):
        super().__init__(output=output, source=source,
                         project_options=project_options,
                         metadata=metadata, container_name=metadata['name'],
                         remote='local')
        self._processes = []

    def _ensure_container(self):
        new_container = not self._get_container_status()
        if new_container:
            try:
                subprocess.check_call([
                    'lxc', 'init', self._image, self._container_name])
            except subprocess.CalledProcessError as e:
                raise errors.ContainerConnectionError(
                    'Failed to setup container')
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
                raise errors.ContainerConnectionError(msg)
        self._wait_for_network()
        if new_container:
            self._container_run(['apt-get', 'update'])
            # Because of https://bugs.launchpad.net/snappy/+bug/1628289
            # Needed to run snapcraft as a snap and build-snaps
            self._container_run(['apt-get', 'install', 'squashfuse', '-y'])
        self._inject_snapcraft(new_container=new_container)

    def _configure_container(self):
        super()._configure_container()
        # Map host user to root (0) inside container
        subprocess.check_call([
            'lxc', 'config', 'set', self._container_name,
            'raw.idmap',
            'both {} {}'.format(os.getenv('SUDO_UID', os.getuid()), 0)])
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
        devices = self._get_container_status()['devices']
        if self._project_folder not in devices:
            logger.info('Mounting {} into container'.format(self._source))
            subprocess.check_call([
                'lxc', 'config', 'device', 'add', self._container_name,
                self._project_folder, 'disk', 'source={}'.format(self._source),
                'path={}'.format(self._project_folder)])

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
