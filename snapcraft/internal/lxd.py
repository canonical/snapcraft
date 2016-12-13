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
from subprocess import CalledProcessError
from time import sleep
import pylxd
import urllib
from ws4py.client import WebSocketBaseClient
from ws4py.manager import WebSocketManager

import petname


logger = logging.getLogger(__name__)

_NETWORK_PROBE_COMMAND = \
    'import urllib.request; urllib.request.urlopen("{}", timeout=5)'.format(
        'http://start.ubuntu.com/connectivity-check.html')
_PROXY_KEYS = ['http_proxy', 'https_proxy', 'no_proxy', 'ftp_proxy']


class _CommandWebsocketclient(WebSocketBaseClient):
    def __init__(self, manager, io, *args, **kwargs):
        self._manager = manager
        self._io = io
        super(_CommandWebsocketclient, self).__init__(*args, **kwargs)

    def handshake_ok(self):
        if self._io == sys.stdin:
            self.close()
            return

        self._manager.add(self)

    def received_message(self, message):
        if self._io == sys.stdin:
            return

        if not message.data:
            self.close()
            self._manager.remove(self)
        if message.encoding:
            msg = message.data.decode(message.encoding)
        else:
            msg = message.data.decode('utf-8')
        self._io.write(msg)


class Cleanbuilder:

    def __init__(self, snap_output, tar_filename, project_options):
        self._snap_output = snap_output
        self._tar_filename = tar_filename
        self._project_options = project_options
        self._container_name = 'snapcraft-{}'.format(
            petname.Generate(3, '-'))
        self._find_lxd()
        self._container = None

    def _find_lxd(self):
        try:
            self._client = pylxd.Client()
        except:
            try:
                os.environ['LXD_DIR'] = '/var/snap/lxd/common/lxd'
                self._client = pylxd.Client()
            except:
                raise EnvironmentError(
                    'The lxd package is not installed, in order to use '
                    '`cleanbuild` you must install lxd onto your system. '
                    'Refer to the "Ubuntu Desktop and Ubuntu Server" section '
                    'on https://linuxcontainers.org/lxd/getting-started-cli/'
                    '#ubuntu-desktop-and-ubuntu-server to enable a proper '
                    'setup.')

    def _push_file(self, src, dst):
        logger.info('Pushing {} to {}'.format(src, dst))
        with open(src, 'rb') as source_file:
            data = source_file.read()
            self._container.files.put(dst, data)

    def _pull_file(self, src, dst):
        logger.info('Pulling {} from {}'.format(dst, src))
        with open(dst, 'wb') as destination_file:
            data = self._container.files.get(src)
            destination_file.write(data)

    def _container_run(self, cmd):
        if not self._container:
            raise CalledProcessError(-1, cmd)
        logger.info('Executing {}'.format(cmd))
        # API call because self._container.execute doesn't expose IO streams
        response = self._container.api.exec.post(json={
            'command': cmd,
            # Make debconf stop asking for user input
            'environment': {
                'DEBIAN_FRONTEND': 'noninteractive',
            },
            'wait-for-websocket': True,
            'interactive': False,
        })
        fds = response.json()['metadata']['metadata']['fds']
        operation_id = response.json()['operation'].split('/')[-1]
        parsed = urllib.parse.urlparse(
            self._client.api.operations[operation_id].websocket._api_endpoint)
        manager = WebSocketManager()
        for io in [sys.stdin, sys.stdout, sys.stderr]:
            pipe = _CommandWebsocketclient(
                manager, io, self._client.websocket_url)
            secret = fds[str(io.fileno())]
            pipe.resource = '{}?secret={}'.format(parsed.path, secret)
            pipe.connect()
        manager.start()
        while len(manager) > 0:
            sleep(.1)
        response = self._client.api.operations[operation_id].get()
        exit_status = response.json()['metadata']['metadata']['return']
        if exit_status is not 0:
            raise CalledProcessError(exit_status, cmd)

    def _get_fingerprint_by_name(self, name):
        for image in self._client.images.all():
            image_name = '{}:{}/{}'.format(
                image.properties['os'],
                image.properties['release'],
                image.properties['architecture'])
            if name == image_name:
                return image.fingerprint
        return 'N/A'

    @contextmanager
    def _create_container(self):
        try:
            fingerprint = self._get_fingerprint_by_name(
                'ubuntu:xenial/{}'.format(self._project_options.deb_arch))
            logger.info('Creating container {} with fingerprint {}'.format(
                self._container_name, fingerprint[:12]))
            self._container = self._client.containers.create({
                'name': self._container_name,
                'ephemeral': True,
                'source': {
                    'type': 'image',
                    'fingerprint': fingerprint,
                }
            }, wait=True)
            self._container.start()
            yield
        finally:
            # Stopping takes a while and lxc doesn't print anything.
            print('Stopping {}'.format(self._container_name))
            if self._container:
                self._container.stop(force=True, wait=True)

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
