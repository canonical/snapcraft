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

import collections
import json
import logging
import os
import pipes
import requests
import requests_unixsocket
import shutil
import sys
import tempfile
import yaml
from contextlib import contextmanager
import subprocess
import time
from urllib import parse

from snapcraft.internal import common
from snapcraft.internal.errors import (
        ContainerConnectionError,
        ContainerRunError,
        ContainerSnapcraftCmdError,
        SnapdError,
)
from snapcraft._options import _get_deb_arch
from snapcraft.internal.repo import snaps

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
        self._server_arch = _get_deb_arch(kernel)
        if not self._server_arch:
            raise ContainerConnectionError(
                'Unrecognized server architecture {}'.format(kernel))
        self._image = 'ubuntu:xenial/{}'.format(self._server_arch)
        # Use a temporary folder the 'lxd' snap can access
        lxd_common_dir = os.path.expanduser(
            os.path.join('~', 'snap', 'lxd', 'common'))
        os.makedirs(lxd_common_dir, exist_ok=True)
        self.tmp_dir = tempfile.mkdtemp(prefix='snapcraft', dir=lxd_common_dir)

    def _get_remote_info(self):
        remote = self._container_name.split(':')[0]
        return yaml.load(subprocess.check_output([
            'lxc', 'info', '{}:'.format(remote)]).decode())

    @contextmanager
    def _container_running(self):
        with self._ensure_started():
            try:
                yield
            except ContainerRunError as e:
                if self._project_options.debug:
                    logger.info('Debug mode enabled, dropping into a shell')
                    self._container_run(['bash', '-i'])
                else:
                    raise e
            else:
                # Remove temporary folder if everything went well
                shutil.rmtree(self.tmp_dir)
                self._finish()

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
                subprocess.check_call([
                    'lxc', 'stop', '-f', self._container_name])

    def _get_container_status(self):
        containers = json.loads(subprocess.check_output([
            'lxc', 'list', '--format=json', self._container_name]).decode())
        for container in containers:
            if container['name'] == self._container_name.split(':')[-1]:
                return container

    def _configure_container(self):
        subprocess.check_call([
            'lxc', 'config', 'set', self._container_name,
            'environment.SNAPCRAFT_SETUP_CORE', '1'])
        if os.getenv('SNAPCRAFT_PARTS_URI'):
            subprocess.check_call([
                'lxc', 'config', 'set', self._container_name,
                'environment.SNAPCRAFT_PARTS_URI',
                os.getenv('SNAPCRAFT_PARTS_URI')])
        # Necessary to read asset files with non-ascii characters.
        subprocess.check_call([
            'lxc', 'config', 'set', self._container_name,
            'environment.LC_ALL', 'C.UTF-8'])
        self._set_image_info_env_var()

    def _set_image_info_env_var(self):
        FAILURE_WARNING_FORMAT = (
            'Failed to get container image info: {}\n'
            'It will not be recorded in manifest.')
        try:
            image_info_command = [
                'lxc', 'image', 'list', '--format=json', self._image]
            image_info = json.loads(subprocess.check_output(
                image_info_command).decode())
        except subprocess.CalledProcessError as e:
            message = ('`{command}` returned with exit code {returncode}, '
                       'output: {output}'.format(
                           command=' '.join(image_info_command),
                           returncode=e.returncode,
                           output=e.output))
            logger.warning(FAILURE_WARNING_FORMAT.format(message))
            return
        except json.decoder.JSONDecodeError as e:
            logger.warning(FAILURE_WARNING_FORMAT.format('Not in JSON format'))
            return
        edited_image_info = collections.OrderedDict()
        for field in ('fingerprint', 'architecture', 'created_at'):
            if field in image_info[0]:
                edited_image_info[field] = image_info[0][field]
        # Pass the image info to the container so it can be used when recording
        # information about the build environment.
        subprocess.check_call([
            'lxc', 'config', 'set', self._container_name,
            'environment.SNAPCRAFT_IMAGE_INFO',
            json.dumps(edited_image_info)])

    def execute(self, step='snap', args=None):
        with self._container_running():
            self._setup_project()
            command = ['snapcraft', step]
            if step == 'snap':
                command += ['--output', self._snap_output]
            # Pass on target arch if specified
            # If not specified it defaults to the LXD architecture
            if self._project_options.target_arch:
                command += ['--target-arch', self._project_options.target_arch]
            if args:
                command += args
            self._container_run(command, cwd=self._project_folder)

    def _container_run(self, cmd, cwd=None, **kwargs):
        sh = ''
        original_cmd = cmd.copy()
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
        try:
            subprocess.check_call([
                'lxc', 'exec', self._container_name, '--'] + cmd,
                **kwargs)
        except subprocess.CalledProcessError as e:
            if original_cmd[0] == 'snapcraft':
                raise ContainerSnapcraftCmdError(command=original_cmd,
                                                 exit_code=e.returncode)
            else:
                raise ContainerRunError(command=original_cmd,
                                        exit_code=e.returncode)

    def _wait_for_network(self):
        logger.info('Waiting for a network connection...')
        not_connected = True
        retry_count = 5
        while not_connected:
            time.sleep(5)
            try:
                self._container_run(['python3', '-c', _NETWORK_PROBE_COMMAND])
                not_connected = False
            except ContainerRunError as e:
                retry_count -= 1
                if retry_count == 0:
                    raise e
        logger.info('Network connection established')

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

        # If the server has a different arch we can't inject local snaps
        if (self._project_options.target_arch
                and self._project_options.target_arch != self._server_arch):
            channel = json['result']['channel']
            return self._install_snap(name, channel, is_classic=is_classic)

        # Revisions are unique, so we don't need to know the channel
        rev = json['result']['revision']

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
            subprocess.check_call(['sudo', 'cp', installed, filepath])
            subprocess.check_call([
                'sudo', 'chown', str(os.getuid()), filepath])
        else:
            shutil.copyfile(installed, filepath)

        if self._is_same_snap(filepath, name):
            logger.debug('Not re-injecting same version of {!r}'.format(name))
            return

        if not rev.startswith('x'):
            self._inject_assertions('{}_{}.assert'.format(name, rev), [
                ['account-key', 'public-key-sha3-384={}'.format(_STORE_KEY)],
                ['snap-declaration', 'snap-name={}'.format(name)],
                ['snap-revision', 'snap-revision={}'.format(rev),
                 'snap-id={}'.format(id)],
            ])

        container_filename = os.path.join(os.sep, 'run', filename)
        self._push_file(filepath, container_filename)
        self._install_snap(container_filename,
                           is_dangerous=rev.startswith('x'),
                           is_classic=is_classic)

    def _pull_file(self, src, dst):
        subprocess.check_call(['lxc', 'file', 'pull',
                               '{}{}'.format(self._container_name, src), dst])

    def _push_file(self, src, dst):
        subprocess.check_call(['lxc', 'file', 'push',
                              src, '{}{}'.format(self._container_name, dst)])

    def _install_snap(self, name, channel=None,
                      is_dangerous=False,
                      is_classic=False):
        logger.info('Installing {}'.format(name))
        # Install: will do nothing if already installed
        args = []
        if channel:
            args.append('--channel')
            args.append(channel)
        if is_dangerous:
            args.append('--dangerous')
        if is_classic:
            args.append('--classic')
        self._container_run(['snap', 'install', name] + args)
        if channel:
            # Switch channel if install was a no-op
            self._container_run(['snap', 'refresh', name] + args)

    def _is_same_snap(self, filepath, name):
        # Compare checksums: user-visible version may still match
        checksum = subprocess.check_output(['sha384sum', filepath]).decode(
            sys.getfilesystemencoding()).split()[0]
        try:
            # Find the current version in use in the container
            rev = subprocess.check_output([
                'lxc', 'exec', self._container_name, '--',
                'readlink', '/snap/{}/current'.format(name)]
                ).decode(sys.getfilesystemencoding()).strip()
            filename = '{}_{}.snap'.format(name, rev)
            installed = os.path.join(os.path.sep,
                                     'var', 'lib', 'snapd', 'snaps', filename)
            checksum_container = subprocess.check_output([
                'lxc', 'exec', self._container_name, '--',
                'sha384sum', installed]
                ).decode(sys.getfilesystemencoding()).split()[0]
        except subprocess.CalledProcessError:
            # Snap not installed
            checksum_container = None
        return checksum == checksum_container

    def _inject_assertions(self, filename, assertions):
        filepath = os.path.join(self.tmp_dir, filename)
        with open(filepath, 'wb') as f:
            for assertion in assertions:
                logger.info('Looking up assertion {}'.format(assertion))
                f.write(subprocess.check_output(['snap', 'known', *assertion]))
                f.write(b'\n')
        container_filename = os.path.join(os.path.sep, 'run', filename)
        self._push_file(filepath, container_filename)
        logger.info('Adding assertion {}'.format(filename))
        self._container_run(['snap', 'ack', container_filename])


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
        default_remote = subprocess.check_output(
            ['lxc', 'remote', 'get-default'])
    except FileNotFoundError:
        raise ContainerConnectionError(
            'You must have LXD installed in order to use cleanbuild.')
    except subprocess.CalledProcessError:
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
        subprocess.check_output(['lxc', 'list', '{}:'.format(remote)])
    except FileNotFoundError:
        raise ContainerConnectionError(
            'You must have LXD installed in order to use cleanbuild.')
    except subprocess.CalledProcessError as e:
        raise ContainerConnectionError(
            'There are either no permissions or the remote {!r} '
            'does not exist.\n'
            'Verify the existing remotes by running `lxc remote list`\n'
            .format(remote)) from e
