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
from subprocess import CalledProcessError
from unittest.mock import (
    call,
    patch,
)

import fixtures
from testtools import ExpectedException

from snapcraft import tests
from snapcraft import ProjectOptions
from snapcraft.internal import lxd
from snapcraft.tests import check_output_side_effect


class LXDTestCase(tests.ContainerTestCase):

    @patch('petname.Generate')
    def test_cleanbuild(self, mock_pet):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        mock_pet.return_value = 'my-pet'

        project_options = ProjectOptions()
        metadata = {'name': 'project'}
        project_folder = 'build_project'
        lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                         metadata=metadata,
                         project_options=project_options).execute()
        expected_arch = project_options.deb_arch

        self.assertEqual(
            'Setting up container with project assets\n'
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
            call(['lxc', 'exec', 'local:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'mkdir', project_folder]),
            call(['lxc', 'file', 'push', os.path.realpath('project.tar'),
                  'local:snapcraft-my-pet/build_project/project.tar']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'tar', 'xvf', 'project.tar']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'python3', '-c',
                  'import urllib.request; '
                  'urllib.request.urlopen('
                  '"http://start.ubuntu.com/connectivity-check.html", '
                  'timeout=5)']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'apt-get', 'update']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'apt-get', 'install', 'snapcraft', '-y']),
            call(['lxc', 'exec', 'local:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'snapcraft', 'snap', '--output', 'snap.snap']),
            call(['lxc', 'file', 'pull',
                  'local:snapcraft-my-pet/{}/snap.snap'.format(project_folder),
                  'snap.snap']),
            call(['lxc', 'stop', '-f', 'local:snapcraft-my-pet']),
        ])

    @patch('snapcraft.internal.lxd.sleep')
    def test_wait_for_network_loops(self, mock_sleep):
        self.check_call_mock.side_effect = CalledProcessError(-1, ['my-cmd'])

        metadata = {'name': 'project'}
        cb = lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                              metadata=metadata,
                              project_options='amd64')

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
        metadata = {'name': 'project'}
        lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                         metadata=metadata,
                         project_options=project_options).execute()

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
        metadata = {'name': 'project'}
        self.assertRaises(
            CalledProcessError,
            lxd.Cleanbuilder(
                output='snap.snap', source='project.tar',
                metadata=metadata,
                project_options=project_options).execute)

        self.assertNotIn(['bash', '-i'], call_list)

    @patch('petname.Generate')
    def test_cleanbuild_with_remote(self, mock_pet):
        mock_pet.return_value = 'my-pet'

        project_options = ProjectOptions()
        metadata = {'name': 'project'}
        project_folder = 'build_project'
        lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                         metadata=metadata,
                         project_options=project_options,
                         remote='my-remote').execute()
        expected_arch = project_options.deb_arch

        self.check_call_mock.assert_has_calls([
            call(['lxc', 'launch', '-e',
                  'ubuntu:xenial/{}'.format(expected_arch),
                  'my-remote:snapcraft-my-pet']),
            call(['lxc', 'config', 'set', 'my-remote:snapcraft-my-pet',
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'mkdir', project_folder]),
            call(['lxc', 'file', 'push', os.path.realpath('project.tar'),
                  'my-remote:snapcraft-my-pet/build_project/project.tar']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'tar', 'xvf', 'project.tar']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'python3', '-c',
                  'import urllib.request; '
                  'urllib.request.urlopen('
                  '"http://start.ubuntu.com/connectivity-check.html", '
                  'timeout=5)']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'apt-get', 'update']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'apt-get', 'install', 'snapcraft', '-y']),
            call(['lxc', 'exec', 'my-remote:snapcraft-my-pet',
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'snapcraft', 'snap', '--output', 'snap.snap']),
            call(['lxc', 'file', 'pull',
                  'my-remote:snapcraft-my-pet/{}/snap.snap'.format(
                      project_folder),
                  'snap.snap']),
            call(['lxc', 'stop', '-f', 'my-remote:snapcraft-my-pet']),
        ])

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_lxc_check_fails(self, mock_sleep, mock_run):
        self.check_output_mock.side_effect = check_output_side_effect(
              fail_on_default=True)

        project_options = ProjectOptions(debug=False)
        metadata = {'name': 'project'}
        with ExpectedException(
                lxd.SnapcraftEnvironmentError,
                'You must have LXD installed in order to use cleanbuild. '
                'However, it is either not installed or not configured '
                'properly.\n'
                'Refer to the documentation at '
                'https://linuxcontainers.org/lxd/getting-started-cli.'):
            lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                             metadata=metadata,
                             project_options=project_options)

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_remote_does_not_exist(self, mock_sleep, mock_run):
        self.check_output_mock.side_effect = check_output_side_effect(
              fail_on_remote=True)

        project_options = ProjectOptions(debug=False)
        metadata = {'name': 'project'}
        with ExpectedException(lxd.SnapcraftEnvironmentError,
                               'There are either.*my-remote.*'):
            lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                             metadata=metadata,
                             project_options=project_options,
                             remote='my-remote')
