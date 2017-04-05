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

import logging
import os
import sys
import tarfile
from contextlib import contextmanager
from subprocess import check_call, check_output, CalledProcessError
from time import sleep

import petname

from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.internal import common

logger = logging.getLogger(__name__)

_NETWORK_PROBE_COMMAND = \
    'import urllib.request; urllib.request.urlopen("{}", timeout=5)'.format(
        'http://start.ubuntu.com/connectivity-check.html')
_PROXY_KEYS = ['http_proxy', 'https_proxy', 'no_proxy', 'ftp_proxy']


def _get_tar_filter(tar_filename):
    def _tar_filter(tarinfo):
        fn = tarinfo.name
        if fn.startswith('./parts/') and not fn.startswith('./parts/plugins'):
            return None
        elif fn in ('./stage', './prime', tar_filename):
            return None
        elif fn.endswith('.snap'):
            return None
        return tarinfo
    return _tar_filter


class Containerbuild:

    def __init__(self, *, output, source, project_options,
                 metadata, container_name, remote=None):
        if not output:
            output = common.format_snap_name(metadata)
        self._snap_output = output
        if '.tar' in source:
            self._tar_filename = source
        else:
            self._tar_filename = '{}_{}_source.tar.bz2'.format(
                metadata['name'], metadata['version'])
            with tarfile.open(self._tar_filename, 'w:bz2') as t:
                t.add(os.path.curdir,
                      filter=_get_tar_filter(self._tar_filename))
        self._project_options = project_options

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

    def _container_run(self, cmd):
        check_call(['lxc', 'exec', self._container_name, '--'] + cmd)

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

    def execute(self):
        with self._ensure_started():
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


class Cleanbuilder(Containerbuild):

    def __init__(self, *, output, source, project_options,
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

    def _ensure_container(self):
        try:
            check_call([
                'lxc', 'start', self._container_name])
        except:
            check_call([
                'lxc', 'launch',
                'ubuntu:xenial/{}'.format(self._project_options.deb_arch),
                self._container_name])
            check_call([
                'lxc', 'config', 'set', self._container_name,
                'environment.SNAPCRAFT_SETUP_CORE', '1'])


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
