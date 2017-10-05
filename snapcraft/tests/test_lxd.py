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
import requests
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
    SnapdError,
    SnapcraftEnvironmentError,
)
from snapcraft._options import _get_deb_arch


class LXDTestCase(tests.TestCase):

    scenarios = [
        ('local', dict(remote='local', target_arch=None, server='x86_64')),
        ('remote', dict(remote='myremote', target_arch=None, server='x86_64')),
        ('cross', dict(remote='local', target_arch='armhf', server='x86_64')),
        ('arm remote', dict(remote='pi', target_arch=None, server='armv7l')),
        ('arm same', dict(remote='pi', target_arch='armhf', server='armv7l')),
        ('arm cross', dict(remote='pi', target_arch='arm64', server='armv7l')),
    ]

    def setUp(self):
        super().setUp()
        self.fake_lxd = tests.fixture_setup.FakeLXD()
        self.useFixture(self.fake_lxd)
        self.fake_lxd.kernel_arch = self.server
        self.fake_filesystem = tests.fixture_setup.FakeFilesystem()
        self.useFixture(self.fake_filesystem)

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
        expected_arch = _get_deb_arch(self.server)

        self.assertIn('Waiting for a network connection...\n'
                      'Network connection established\n'
                      'Setting up container with project assets\n'
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
            call(['python3', '-c', 'import urllib.request; ' +
                  'urllib.request.urlopen(' +
                  '"http://start.ubuntu.com/connectivity-check.html"' +
                  ', timeout=5)']),
            call(['apt-get', 'update']),
            call(['mkdir', project_folder]),
            call(['tar', 'xvf', 'project.tar'],
                 cwd=project_folder),
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

    @patch('snapcraft.internal.common.is_snap')
    def test_parallel_invocation(self, mock_is_snap):
        mock_is_snap.side_effect = lambda: False

        builder1 = self.make_cleanbuilder()
        builder2 = self.make_cleanbuilder()
        builder1.execute()
        # Temporary folder should be removed in the end
        self.fake_filesystem.rmtree_mock.assert_has_calls([
            call(builder1.tmp_dir)])
        builder2.execute()
        self.fake_filesystem.rmtree_mock.assert_has_calls([
            call(builder2.tmp_dir)])

    @patch('snapcraft.internal.common.is_snap')
    def test_parallel_invocation_inject_snap(self, mock_is_snap):
        mock_is_snap.side_effect = lambda: True

        fake_snapd = tests.fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps_result = [
            {'name': 'core',
             'confinement': 'strict',
             'id': '2kkitQurgOkL3foImG4wDwn9CIANuHlt',
             'revision': '123'},
            {'name': 'snapcraft',
             'confinement': 'classic',
             'id': '3lljuRvshPlM4gpJnH5xExo0DJBOvImu',
             'revision': '345'},
        ]

        builder1 = self.make_cleanbuilder()
        builder2 = self.make_cleanbuilder()
        builder1.execute()
        # Temporary folder should be removed in the end
        self.fake_filesystem.rmtree_mock.assert_has_calls([
            call(builder1.tmp_dir)])
        builder2.execute()
        self.fake_filesystem.rmtree_mock.assert_has_calls([
            call(builder2.tmp_dir)])

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @patch('snapcraft.internal.common.is_snap')
    def test_inject_apt(self,
                        mock_is_snap,
                        mock_container_run):
        mock_is_snap.side_effect = lambda: False

        fake_snapd = tests.fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)

        builder = self.make_cleanbuilder()
        builder.execute()

        mock_container_run.assert_has_calls([
            call(['apt-get', 'install', 'snapcraft', '-y']),
        ])

    @patch('snapcraft.internal.common.is_snap')
    def test_inject_socket_error(self,
                                 mock_is_snap):
        mock_is_snap.side_effect = lambda: True

        def snap_details(handler_instalce, snap_name):
            raise requests.exceptions.ConnectionError(
                'Connection aborted.',
                FileNotFoundError(2, 'No such file or directory'))

        fake_snapd = tests.fixture_setup.FakeSnapd()
        fake_snapd.snap_details_func = snap_details
        self.useFixture(fake_snapd)

        builder = self.make_cleanbuilder()
        self.assertIn('Error connecting to',
                      str(self.assertRaises(SnapdError,
                                            builder.execute)))
        # Temporary folder should remain in case of failure
        self.fake_filesystem.rmtree_mock.assert_not_called()

    @patch('snapcraft.internal.common.is_snap')
    def test_inject_snap_api_error(self,
                                   mock_is_snap):
        mock_is_snap.side_effect = lambda: True

        fake_snapd = tests.fixture_setup.FakeSnapd()
        fake_snapd.snaps_result = []
        self.useFixture(fake_snapd)

        builder = self.make_cleanbuilder()
        self.assertIn('Error querying \'core\' snap: not found',
                      str(self.assertRaises(SnapdError,
                                            builder.execute)))
        # Temporary folder should remain in case of failure
        self.fake_filesystem.rmtree_mock.assert_not_called()

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @patch('snapcraft.internal.common.is_snap')
    def test_inject_snap(self,
                         mock_is_snap,
                         mock_container_run):
        mock_is_snap.side_effect = lambda: True
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        fake_snapd = tests.fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps_result = [
            {'name': 'core',
             'confinement': 'strict',
             'id': '2kkitQurgOkL3foImG4wDwn9CIANuHlt',
             'revision': '123'},
            {'name': 'snapcraft',
             'confinement': 'classic',
             'id': '3lljuRvshPlM4gpJnH5xExo0DJBOvImu',
             'revision': '345'},
        ]

        builder = self.make_cleanbuilder()

        builder.execute()
        self.fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'file', 'push',
                  os.path.join(builder.tmp_dir, 'core_123.assert'),
                  '{}/run/core_123.assert'.format(self.fake_lxd.name)]),
            call(['lxc', 'file', 'push',
                  os.path.join(builder.tmp_dir, 'core_123.snap'),
                  '{}/run/core_123.snap'.format(self.fake_lxd.name)]),
            call(['lxc', 'file', 'push',
                  os.path.join(builder.tmp_dir, 'snapcraft_345.assert'),
                  '{}/run/snapcraft_345.assert'.format(self.fake_lxd.name)]),
            call(['lxc', 'file', 'push',
                  os.path.join(builder.tmp_dir, 'snapcraft_345.snap'),
                  '{}/run/snapcraft_345.snap'.format(self.fake_lxd.name)]),
        ])
        mock_container_run.assert_has_calls([
            call(['apt-get', 'install', 'squashfuse', '-y']),
            call(['snap', 'ack', '/run/core_123.assert']),
            call(['snap', 'install', '/run/core_123.snap']),
            call(['snap', 'ack', '/run/snapcraft_345.assert']),
            call(['snap', 'install', '/run/snapcraft_345.snap', '--classic']),
        ])

    @patch('os.getuid')
    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @patch('snapcraft.internal.common.is_snap')
    def test_inject_snap_dangerous(self,
                                   mock_is_snap,
                                   mock_container_run,
                                   mock_getuid):
        mock_is_snap.side_effect = lambda: True
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        mock_getuid.return_value = 1234

        fake_snapd = tests.fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps_result = [
            {'name': 'core',
             'confinement': 'strict',
             'id': '2kkitQurgOkL3foImG4wDwn9CIANuHlt',
             'revision': '123'},
            {'name': 'snapcraft',
             'confinement': 'classic',
             'id': '',
             'revision': 'x1'},
        ]

        builder = self.make_cleanbuilder()

        builder.execute()
        self.fake_lxd.check_call_mock.assert_has_calls([
            call(['sudo', 'cp', '/var/lib/snapd/snaps/snapcraft_x1.snap',
                  os.path.join(builder.tmp_dir, 'snapcraft_x1.snap')]),
            call(['sudo', 'chown', str(os.getuid()),
                  os.path.join(builder.tmp_dir, 'snapcraft_x1.snap')]),
            call(['lxc', 'file', 'push',
                  os.path.join(builder.tmp_dir, 'snapcraft_x1.snap'),
                  '{}/run/snapcraft_x1.snap'.format(self.fake_lxd.name)]),
        ])
        mock_container_run.assert_has_calls([
            call(['snap', 'install', '/run/snapcraft_x1.snap',
                  '--dangerous', '--classic']),
        ])


class ProjectTestCase(LXDTestCase):

    scenarios = [
        ('remote', dict(remote='myremote', target_arch=None, server='x86_64')),
    ]

    def make_project(self):
        return lxd.Project(output='snap.snap', source='project.tar',
                           metadata={'name': 'project'},
                           project_options=self.project_options,
                           remote=self.remote)

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    def test_start_failed(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        def call_effect(*args, **kwargs):
            if args[0][:2] == ['lxc', 'start']:
                raise CalledProcessError(
                    returncode=255, cmd=args[0])
            return d(*args, **kwargs)

        d = self.fake_lxd.check_call_mock.side_effect
        self.fake_lxd.check_call_mock.side_effect = call_effect

        self.assertIn(
            'The container could not be started.\n',
            str(self.assertRaises(
                ContainerConnectionError,
                self.make_project().execute)))

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    def test_ftp_not_installed(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        def call_effect(*args, **kwargs):
            if args[0][:1] == ['/usr/lib/sftp-server']:
                raise FileNotFoundError(
                    2, 'No such file or directory')

        self.fake_lxd.popen_mock.side_effect = call_effect

        self.assertIn(
            'You must have openssh-sftp-server installed to use a LXD '
            'remote on a different host.\n',
            str(self.assertRaises(
                SnapcraftEnvironmentError,
                self.make_project().execute)))

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    def test_ftp_error(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        def call_effect(*args, **kwargs):
            if args[0][:1] == ['/usr/lib/sftp-server']:
                raise CalledProcessError(
                    returncode=255, cmd=args[0])

        self.fake_lxd.popen_mock.side_effect = call_effect

        self.assertIn(
            'sftp-server seems to be installed but could not be run.\n',
            str(self.assertRaises(
                SnapcraftEnvironmentError,
                self.make_project().execute)))

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    def test_sshfs_failed(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        def call_effect(*args, **kwargs):
            if args[0][:2] == ['lxc', 'exec'] and args[0][4] == 'ls':
                return ''.encode('utf-8')
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_output_mock.side_effect = call_effect

        self.assertIn(
            'The project folder could not be mounted.\n',
            str(self.assertRaises(
                ContainerConnectionError,
                self.make_project().execute)))


class LocalProjectTestCase(LXDTestCase):

    scenarios = [
        ('local', dict(remote='local', target_arch=None, server='x86_64')),
    ]

    def make_project(self):
        return lxd.Project(output='snap.snap', source='project.tar',
                           metadata={'name': 'project'},
                           project_options=self.project_options,
                           remote=self.remote)

    @patch('snapcraft.internal.lxd.Containerbuild._container_run')
    def test_start_failed(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        def call_effect(*args, **kwargs):
            if args[0][:2] == ['lxc', 'start']:
                raise CalledProcessError(
                    returncode=255, cmd=args[0])
            return d(*args, **kwargs)

        d = self.fake_lxd.check_call_mock.side_effect
        self.fake_lxd.check_call_mock.side_effect = call_effect

        self.assertIn(
            'The container could not be started.\n'
            'The files /etc/subuid and /etc/subgid need to contain this line ',
            str(self.assertRaises(
                ContainerConnectionError,
                self.make_project().execute)))
        # Should not attempt to stop a container that wasn't started
        self.assertNotIn(call(['lxc', 'stop', '-f', self.fake_lxd.name]),
                         self.fake_lxd.check_call_mock.call_args_list)
