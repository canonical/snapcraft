#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
from contextlib import contextmanager
from subprocess import check_call, CalledProcessError
from time import sleep

import petname

from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.internal import cache


logger = logging.getLogger(__name__)

_NETWORK_PROBE_COMMAND = \
    'import urllib.request; urllib.request.urlopen("{}", timeout=5)'.format(
        'http://start.ubuntu.com/connectivity-check.html')
_PROXY_KEYS = ['http_proxy', 'https_proxy', 'no_proxy', 'ftp_proxy']


class Cleanbuilder:

    def __init__(self, snap_output, tar_filename, project_options,
                 remote=None):
        self._snap_output = snap_output
        self._tar_filename = tar_filename
        self._project_options = project_options
        container_name = 'snapcraft-{}'.format(petname.Generate(3, '-'))
        if remote:
            self._verify_remote(remote)
            self._container_name = '{}:{}'.format(remote, container_name)
        else:
            self._container_name = container_name

    def _verify_remote(self, remote):
        # There is no easy way to grep the results from `lxc remote list`
        # so we try and execute a simple operation against the remote.
        try:
            check_call(['lxc', 'list', '{}:'.format(remote)])
        except CalledProcessError as e:
            raise SnapcraftEnvironmentError(
                'There are either no permissions or the remote {!r} '
                'does not exist.\n'
                'Verify the existing remotes by running `lxc remote list`\n'
                'To setup a new remote, follow the instructions on\n'
                'https://linuxcontainers.org/lxd/getting-started-cli/'
                '#multiple-hosts'.format(remote)) from e

    def _push_file(self, src, dst):
        check_call(['lxc', 'file', 'push',
                    src, '{}/{}'.format(self._container_name, dst)])

    def _pull_file(self, src, dst):
        check_call(['lxc', 'file', 'pull',
                    '{}/{}'.format(self._container_name, src), dst])

    def _container_run(self, cmd):
        check_call(['lxc', 'exec', self._container_name, '--'] + cmd)

    @contextmanager
    def _create_container(self):
        try:
            check_call([
                'lxc', 'launch', '-e',
                'ubuntu:xenial/{}'.format(self._project_options.deb_arch),
                self._container_name])
            check_call([
                'lxc', 'config', 'set', self._container_name,
                'environment.SNAPCRAFT_SETUP_CORE', '1'])
            yield
        finally:
            # Stopping takes a while and lxc doesn't print anything.
            print('Stopping {}'.format(self._container_name))
            check_call(['lxc', 'stop', '-f', self._container_name])

    def execute(self):
        with self._create_container():
            self._setup_project()
            self._wait_for_network()
            self._container_run(['apt-get', 'update'])
            self._container_run(['apt-get', 'install', 'snapcraft', '-y'])
            try:
                self._container_run(
                    ['snapcraft', 'snap', '--output', self._snap_output])
            except CalledProcessError as e:
                if self._project_options.debug:
                    logger.info('Debug mode enabled, dropping into a shell')
                    self._container_run(['bash', '-i'])
                else:
                    raise e
            else:
                self._pull_snap()

    def _setup_project(self):
        logger.info('Setting up container with project assets')
        dst = os.path.join('/root', os.path.basename(self._tar_filename))
        self._push_file(self._tar_filename, dst)
        self._container_run(['tar', 'xvf', dst])

        logger.info('Copying snapcraft cache into container')
        snapcraft_cache = cache.SnapcraftCache()
        # Later versions have a `push --recursive`. For now we have to do it
        # the hard way and walk the cache ourselves.
        for root, dirs, files in os.walk(snapcraft_cache.cache_root):
            destination_root = os.path.join(
                '/root', '.cache', 'snapcraft', os.path.relpath(
                    root, snapcraft_cache.cache_root))
            self._container_run(['mkdir', '-p', destination_root])
            for file_path in files:
                source = os.path.join(root, file_path)
                destination = os.path.join(destination_root, file_path)
                self._push_file(source, destination)

    def _pull_snap(self):
        src = os.path.join('/root', self._snap_output)
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
