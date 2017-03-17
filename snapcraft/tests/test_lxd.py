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

import os
import logging
from subprocess import CalledProcessError
from unittest.mock import (
    call,
    patch,
)

import fixtures
from testtools import ExpectedException

from snapcraft import tests
from snapcraft import ProjectOptions
from snapcraft.internal import (
    cache,
    lxd,
)


def check_output_side_effect(fail_on_remote=False, fail_on_default=False):
    def call_effect(*args, **kwargs):
        if args[0] == ['lxc', 'remote', 'get-default']:
            if fail_on_default:
                raise CalledProcessError(returncode=255, cmd=args[0])
            else:
                return 'local'.encode('utf-8')
        elif args[0] == ['lxc', 'list', 'my-remote:'] and fail_on_remote:
            raise CalledProcessError(returncode=255, cmd=args[0])
        else:
            return ''.encode('utf-8')
    return call_effect


class LXDTestCase(tests.TestCase):

    def setUp(self):
            super().setUp()

            patcher = patch('snapcraft.internal.lxd.check_call')
            self.check_call_mock = patcher.start()
            self.addCleanup(patcher.stop)

            patcher = patch('snapcraft.internal.lxd.check_output')
            self.check_output_mock = patcher.start()
            self.check_output_mock.side_effect = check_output_side_effect()
            self.addCleanup(patcher.stop)

            patcher = patch('snapcraft.internal.lxd.sleep', lambda _: None)
            patcher.start()
            self.addCleanup(patcher.stop)

    @patch('petname.Generate')
    def test_cleanbuild(self, mock_pet):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        mock_pet.return_value = 'my-pet'

        project_options = ProjectOptions()
        lxd.Cleanbuilder('snap.snap', 'project.tar', project_options).execute()
        expected_arch = project_options.deb_arch

        self.assertEqual(
            'Setting up container with project assets\n'
            'Copying snapcraft cache into container\n'
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Retrieved snap.snap\n',
            fake_logger.output)

        self.check_call_mock.assert_has_calls([
            call(['lxc', 'launch', '-e',
                  'ubuntu:xenial/{}'.format(expected_arch),
                  'local:snapcraft-my-pet']),
            call(['lxc', 'config', 'set', 'local:snapcraft-my-pet',
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'file', 'push', 'project.tar',
                  'local:snapcraft-my-pet//root/project.tar']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'tar', 'xvf', '/root/project.tar']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'python3', '-c',
                  'import urllib.request; '
                  'urllib.request.urlopen('
                  '"http://start.ubuntu.com/connectivity-check.html", '
                  'timeout=5)']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'apt-get', 'update']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'apt-get', 'install', 'snapcraft', '-y']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'snapcraft', 'snap', '--output', 'snap.snap']),
            call(['lxc', 'file', 'pull',
                  'local:snapcraft-my-pet//root/snap.snap',
                  'snap.snap']),
            call(['lxc', 'stop', '-f', 'local:snapcraft-my-pet']),
        ])

    @patch('petname.Generate')
    def test_cleanbuild_copies_cache(self, mock_pet):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        mock_pet.return_value = 'my-pet'

        cache_dir = cache.SnapcraftCache().cache_root
        os.makedirs(cache_dir)
        open(os.path.join(cache_dir, 'foo'), 'w').close()

        project_options = ProjectOptions()
        lxd.Cleanbuilder('snap.snap', 'project.tar', project_options).execute()
        expected_arch = project_options.deb_arch

        self.assertEqual(
            'Setting up container with project assets\n'
            'Copying snapcraft cache into container\n'
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Retrieved snap.snap\n',
            fake_logger.output)

        self.check_call_mock.assert_has_calls([
            call(['lxc', 'launch', '-e',
                  'ubuntu:xenial/{}'.format(expected_arch),
                  'local:snapcraft-my-pet']),
            call(['lxc', 'config', 'set', 'local:snapcraft-my-pet',
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'file', 'push', 'project.tar',
                  'local:snapcraft-my-pet//root/project.tar']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'tar', 'xvf', '/root/project.tar']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'mkdir', '-p', '/root/.cache/snapcraft/.']),
            call(['lxc', 'file', 'push', os.path.join(cache_dir, 'foo'),
                  'local:snapcraft-my-pet//root/.cache/snapcraft/./foo']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'python3', '-c',
                  'import urllib.request; '
                  'urllib.request.urlopen('
                  '"http://start.ubuntu.com/connectivity-check.html", '
                  'timeout=5)']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'apt-get', 'update']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'apt-get', 'install', 'snapcraft', '-y']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet', '--',
                  'snapcraft', 'snap', '--output', 'snap.snap']),
            call(['lxc', 'file', 'pull',
                  'local:snapcraft-my-pet//root/snap.snap',
                  'snap.snap']),
            call(['lxc', 'stop', '-f', 'local:snapcraft-my-pet']),
        ])

    @patch('snapcraft.internal.lxd.sleep')
    def test_wait_for_network_loops(self, mock_sleep):
        self.check_call_mock.side_effect = CalledProcessError(-1, ['my-cmd'])

        cb = lxd.Cleanbuilder('snap.snap', 'project.tar', 'amd64')

        raised = self.assertRaises(
            CalledProcessError,
            cb._wait_for_network)

        self.assertEqual(
            str(raised),
            "Command '['my-cmd']' returned non-zero exit status -1")

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_failed_build_with_debug(self, mock_sleep, mock_run):
        call_list = []

        def run_effect(*args, **kwargs):
            call_list.append(args[0])
            if args[0] == ['snapcraft', 'snap', '--output', 'snap.snap']:
                raise CalledProcessError(returncode=255, cmd=args[0])

        mock_run.side_effect = run_effect

        project_options = ProjectOptions(debug=True)
        lxd.Cleanbuilder('snap.snap', 'project.tar', project_options).execute()

        self.assertIn(['bash', '-i'], call_list)

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_failed_build_without_debug(self, mock_sleep, mock_run):
        call_list = []

        def run_effect(*args, **kwargs):
            call_list.append(args[0])
            if args[0] == ['snapcraft', 'snap', '--output', 'snap.snap']:
                raise CalledProcessError(returncode=255, cmd=args[0])

        mock_run.side_effect = run_effect

        project_options = ProjectOptions(debug=False)
        self.assertRaises(
            CalledProcessError,
            lxd.Cleanbuilder(
                'snap.snap', 'project.tar',
                project_options).execute)

        self.assertNotIn(['bash', '-i'], call_list)

    @patch('petname.Generate')
    def test_cleanbuild_with_remote(self, mock_pet):
        mock_pet.return_value = 'my-pet'

        project_options = ProjectOptions()
        lxd.Cleanbuilder('snap.snap', 'project.tar', project_options,
                         remote='my-remote').execute()
        expected_arch = project_options.deb_arch

        self.check_call_mock.assert_has_calls([
            call(['lxc', 'launch', '-e',
                  'ubuntu:xenial/{}'.format(expected_arch),
                  'my-remote:snapcraft-my-pet']),
            call(['lxc', 'config', 'set', 'my-remote:snapcraft-my-pet',
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'file', 'push', 'project.tar',
                  'my-remote:snapcraft-my-pet//root/project.tar']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet', '--',
                  'tar', 'xvf', '/root/project.tar']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet', '--',
                  'python3', '-c',
                  'import urllib.request; '
                  'urllib.request.urlopen('
                  '"http://start.ubuntu.com/connectivity-check.html", '
                  'timeout=5)']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet', '--',
                  'apt-get', 'update']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet', '--',
                  'apt-get', 'install', 'snapcraft', '-y']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet', '--',
                  'snapcraft', 'snap', '--output', 'snap.snap']),
            call(['lxc', 'file', 'pull',
                  'my-remote:snapcraft-my-pet//root/snap.snap',
                  'snap.snap']),
            call(['lxc', 'stop', '-f', 'my-remote:snapcraft-my-pet']),
        ])

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_lxc_check_fails(self, mock_sleep, mock_run):
        self.check_output_mock.side_effect = check_output_side_effect(
              fail_on_default=True)

        project_options = ProjectOptions(debug=False)
        with ExpectedException(lxd.SnapcraftEnvironmentError,
                               'The lxd package is not installed, in order '
                               'to use `cleanbuild` you must install lxd '
                               'onto your system. Refer to the '
                               '"Ubuntu Desktop and Ubuntu Server" '
                               'section on '
                               'https://linuxcontainers.org/lxd/getting-'
                               'started-cli/#ubuntu-desktop-and-ubuntu-'
                               'server to enable a proper setup.'):
            lxd.Cleanbuilder('snap.snap', 'project.tar',
                             project_options)

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_remote_does_not_exist(self, mock_sleep, mock_run):
        self.check_output_mock.side_effect = check_output_side_effect(
              fail_on_remote=True)

        project_options = ProjectOptions(debug=False)
        with ExpectedException(lxd.SnapcraftEnvironmentError,
                               'There are either.*my-remote.*'):
            lxd.Cleanbuilder('snap.snap', 'project.tar',
                             project_options, remote='my-remote')
