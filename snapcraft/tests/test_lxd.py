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


class LXDTestCase(tests.TestCase):

    scenarios = [
        ('local', dict(remote='local', target_arch=None)),
        ('remote', dict(remote='my-remote', target_arch=None)),
        ('cross', dict(remote='local', target_arch='armhf')),
    ]

    @patch('petname.Generate')
    @patch('platform.machine')
    @patch('platform.architecture')
    def test_cleanbuild(self, mock_arch, mock_machine, mock_pet):
        fake_lxd = tests.fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        mock_pet.return_value = 'my-pet'
        mock_arch.return_value = ('64bit', 'ELF')
        mock_machine.return_value = 'x86_64'
        project_options = ProjectOptions(target_deb_arch=self.target_arch)
        metadata = {'name': 'project'}
        project_folder = 'build_project'
        lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                         metadata=metadata, remote=self.remote,
                         project_options=project_options).execute()
        expected_arch = 'amd64'

        self.assertIn('Setting up container with project assets\n'
                      'Waiting for a network connection...\n'
                      'Network connection established\n'
                      'Retrieved snap.snap\n', fake_logger.output)
        args = []
        if self.target_arch:
            self.assertIn('Setting target machine to \'{}\'\n'.format(
                          self.target_arch), fake_logger.output)
            args += ['--target-arch', self.target_arch]

        container_name = '{}:snapcraft-my-pet'.format(self.remote)
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'launch', '-e',
                  'ubuntu:xenial/{}'.format(expected_arch), container_name]),
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.LC_ALL', 'en_US.UTF-8']),
            call(['lxc', 'exec', container_name,
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'mkdir', project_folder]),
            call(['lxc', 'file', 'push', os.path.realpath('project.tar'),
                  '{}/build_project/project.tar'.format(container_name)]),
            call(['lxc', 'exec', container_name,
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'tar', 'xvf', 'project.tar']),
            call(['lxc', 'exec', container_name,
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'python3', '-c',
                  'import urllib.request; '
                  'urllib.request.urlopen('
                  '"http://start.ubuntu.com/connectivity-check.html", '
                  'timeout=5)']),
            call(['lxc', 'exec', container_name,
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'apt-get', 'update']),
            call(['lxc', 'exec', container_name,
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'apt-get', 'install', 'snapcraft', '-y']),
            call(['lxc', 'exec', container_name,
                  '--env', 'HOME=/{}'.format(project_folder), '--',
                  'snapcraft', 'snap', '--output', 'snap.snap', *args]),
            call(['lxc', 'file', 'pull',
                  '{}/{}/snap.snap'.format(container_name, project_folder),
                  'snap.snap']),
            call(['lxc', 'stop', '-f', container_name]),
        ])

    def test_wait_for_network_loops(self):
        fake_lxd = tests.fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        fake_lxd.check_call_mock.side_effect = CalledProcessError(
            -1, ['my-cmd'])

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
    def test_failed_build_with_debug(self, mock_run):
        self.useFixture(tests.fixture_setup.FakeLXD())
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
    def test_failed_build_without_debug(self, mock_run):
        self.useFixture(tests.fixture_setup.FakeLXD())
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

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    def test_lxc_check_fails(self, mock_run):
        self.useFixture(tests.fixture_setup.FakeLXD(fail_on_default=True))

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
    def test_remote_does_not_exist(self, mock_run):
        self.useFixture(tests.fixture_setup.FakeLXD(fail_on_remote=True))

        project_options = ProjectOptions(debug=False)
        metadata = {'name': 'project'}
        with ExpectedException(lxd.SnapcraftEnvironmentError,
                               'There are either.*my-remote.*'):
            lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                             metadata=metadata,
                             project_options=project_options,
                             remote='my-remote')
