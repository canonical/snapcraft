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
import sys
from contextlib import contextmanager
from subprocess import check_call, CalledProcessError
from time import sleep
import pylxd

import petname
from snapcraft.internal import (
    common,
    repo,
)


logger = logging.getLogger(__name__)

_NETWORK_PROBE_COMMAND = \
    'import urllib.request; urllib.request.urlopen("{}", timeout=5)'.format(
        'http://start.ubuntu.com/connectivity-check.html')
_PROXY_KEYS = ['http_proxy', 'https_proxy', 'no_proxy', 'ftp_proxy']


class Cleanbuilder:

    def __init__(self, snap_output, tar_filename, project_options):
        self._snap_output = snap_output
        self._tar_filename = tar_filename
        self._project_options = project_options
        self._container_name = 'snapcraft-{}'.format(
            petname.Generate(3, '-'))

    def _has_snap(self, snapname):
        try:
            common.run(['snap', 'list', snapname])
            return True
        except:
            return False

    def _find_lxd(self):
        # FIXME: lp#1583259 Do this in snapcraft.yaml
        if 'SNAP_NAME' in os.environ or self._has_snap('lxd'):
            os.environ['LXD_DIR'] = '/var/snap/lxd/common/lxd'
        elif repo.is_package_installed('lxd'):
            pass
        else:
            raise EnvironmentError(
                'The lxd package is not installed, in order to use `cleanbuild` '
                'you must install lxd onto your system. Refer to the '
                '"Ubuntu Desktop and Ubuntu Server" section on '
                'https://linuxcontainers.org/lxd/getting-started-cli/'
                '#ubuntu-desktop-and-ubuntu-server to enable a proper setup.')

    def _push_file(self, src, dst):
        client = pylxd.Client()
        container = client.containers.get(self._container_name)
        print('Pushed {} to {}'.format(src, dst))
        with open(src, 'rb') as source_file:
            data = source_file.read()
            container.files.put(dst, data)

    def _pull_file(self, src, dst):
        client = pylxd.Client()
        container = client.containers.get(self._container_name)
        print('Pulled {} from {}'.format(dst, src))
        with open(dst, 'wb') as destination_file:
            data = container.files.get(src)
            destination_file.write(data)

    def _container_run(self, cmd):
        client = pylxd.Client()
        container = client.containers.get(self._container_name)
        print('Executing {}'.format(cmd))
        container.execute(cmd)

    @contextmanager
    def _create_container(self):
        client = pylxd.Client()
        try:
            # ubuntu:xenial x86-64
            container = client.containers.create(
                { 'name': self._container_name,
                  'ephemeral': True,
                # 'architecture': self._project_options.deb_arch,
                    'source': {
                        'type': 'image',
                        'fingerprint': '315bedd32580',
                    }
                }, wait=True)
            container.start()
            yield
        finally:
            # Stopping takes a while and lxc doesn't print anything.
            print('Stopping {}'.format(self._container_name))
            container = client.containers.get(self._container_name)
            container.stop(force=True, wait=True)

    def execute(self):
        self._find_lxd()
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
