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
from testtools.matchers import Contains, Equals

from snapcraft import tests
from snapcraft import ProjectOptions
from snapcraft.internal import lxd
from snapcraft.internal.errors import (
    ContainerConnectionError,
)


class LXDTestCase(tests.TestCase):

    scenarios = [
        ('local', dict(remote='local', target_arch=None)),
        ('remote', dict(remote='my-remote', target_arch=None)),
        ('cross', dict(remote='local', target_arch='armhf')),
    ]

    def setUp(self):
        super().setUp()
        self.fake_lxd = tests.fixture_setup.FakeLXD()
        self.useFixture(self.fake_lxd)
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.project_options = ProjectOptions(target_deb_arch=self.target_arch)

    def make_cleanbuilder(self):
        return lxd.Cleanbuilder(output='snap.snap', source='project.tar',
                                metadata={'name': 'project'},
                                project_options=self.project_options,
                                remote=self.remote)

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @patch('snapcraft.internal.lxd.Containerbuild._inject_snapcraft')
    @patch('petname.Generate')
    def test_cleanbuild(self, mock_pet, mock_inject, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        mock_pet.return_value = 'my-pet'
        project_folder = '/root/build_project'
        self.make_cleanbuilder().execute()
        expected_arch = 'amd64'

        self.assertIn('Setting up container with project assets\n'
                      'Waiting for a network connection...\n'
                      'Network connection established\n'
                      'Retrieved snap.snap\n', self.fake_logger.output)
        args = []
        if self.target_arch:
            self.assertIn('Setting target machine to \'{}\'\n'.format(
                          self.target_arch), self.fake_logger.output)
            args += ['--target-arch', self.target_arch]

        container_name = '{}:snapcraft-my-pet'.format(self.remote)
        self.fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'launch', '-e',
                  'ubuntu:xenial/{}'.format(expected_arch), container_name]),
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.LC_ALL', 'C.UTF-8']),
            call(['lxc', 'file', 'push', os.path.realpath('project.tar'),
                  '{}/root/build_project/project.tar'.format(container_name)]),
        ])
        mock_container_run.assert_has_calls([
            call(['mkdir', project_folder]),
            call(['tar', 'xvf', 'project.tar'],
                 cwd=project_folder),
            call(['python3', '-c', 'import urllib.request; ' +
                  'urllib.request.urlopen(' +
                  '"http://start.ubuntu.com/connectivity-check.html"' +
                  ', timeout=5)']),
            call(['apt-get', 'update']),
            call(['snapcraft', 'snap', '--output', 'snap.snap', *args],
                 cwd=project_folder),
        ])
        self.fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'file', 'pull',
                  '{}{}/snap.snap'.format(container_name, project_folder),
                  'snap.snap']),
            call(['lxc', 'stop', '-f', container_name]),
        ])

    def test_wait_for_network_loops(self):
        self.fake_lxd.check_call_mock.side_effect = CalledProcessError(
            -1, ['my-cmd'])

        builder = self.make_cleanbuilder()

        raised = self.assertRaises(
            CalledProcessError,
            builder._wait_for_network)

        self.assertThat(str(raised), Contains("Command '['my-cmd']'"))

    def test_failed_container_never_created(self):
        def call_effect(*args, **kwargs):
            if args[0][:2] == ['lxc', 'launch']:
                raise CalledProcessError(returncode=255, cmd=args[0])
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_call_mock.side_effect = call_effect

        raised = self.assertRaises(
            CalledProcessError,
            self.make_cleanbuilder().execute)
        self.assertThat(self.fake_lxd.status, Equals(None))
        # lxc launch should fail and no further commands should come after that
        self.assertThat(str(raised), Contains("Command '['lxc', 'launch'"))

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    def test_failed_build_with_debug(self, mock_run):
        call_list = []

        def run_effect(*args, **kwargs):
            call_list.append(args[0])
            if args[0][:4] == ['snapcraft', 'snap', '--output', 'snap.snap']:
                raise CalledProcessError(returncode=255, cmd=args[0])

        mock_run.side_effect = run_effect

        self.project_options = ProjectOptions(debug=True)
        self.make_cleanbuilder().execute()

        self.assertIn(['bash', '-i'], call_list)

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    def test_failed_build_without_debug(self, mock_run):
        call_list = []

        def run_effect(*args, **kwargs):
            call_list.append(args[0])
            if args[0][:4] == ['snapcraft', 'snap', '--output', 'snap.snap']:
                raise CalledProcessError(returncode=255, cmd=args[0])

        mock_run.side_effect = run_effect

        self.assertRaises(
            CalledProcessError,
            self.make_cleanbuilder().execute)

        self.assertNotIn(['bash', '-i'], call_list)

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    def test_lxc_check_fails(self, mock_run):
        self.fake_lxd.check_output_mock.side_effect = FileNotFoundError('lxc')

        with ExpectedException(
                ContainerConnectionError,
                'You must have LXD installed in order to use cleanbuild.\n'
                'Refer to the documentation at '
                'https://linuxcontainers.org/lxd/getting-started-cli.'):
            self.make_cleanbuilder()

    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    def test_remote_does_not_exist(self, mock_run):
        self.fake_lxd.check_output_mock.side_effect = CalledProcessError(
            255, ['lxd', 'list', self.remote])

        with ExpectedException(ContainerConnectionError,
                               'There are either.*{}.*'.format(self.remote)):
            self.make_cleanbuilder()
