# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
import os.path
import subprocess
from textwrap import dedent
import requests
from unittest import mock
from unittest.mock import call
import snapcraft.internal.errors

import fixtures
from testtools.matchers import (
    Contains,
    Equals,
    FileContains,
    FileExists,
    Not,
)
from . import CommandBaseTestCase
from snapcraft.tests import fixture_setup
from snapcraft.internal.errors import SnapcraftEnvironmentError


class SnapCommandBaseTestCase(CommandBaseTestCase):

    yaml_template = dedent("""\
        name: snap-test
        version: 1.0
        summary: test snapping
        description: if snap is succesful a snap package will be available
        architectures: ['amd64']
        type: {}
        confinement: strict
        grade: stable

        parts:
            part1:
                plugin: nil
        """)

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft.internal.indicators.is_dumb_terminal')
        dumb_mock = patcher.start()
        dumb_mock.return_value = True
        self.addCleanup(patcher.stop)

        self.useFixture(fixture_setup.FakeTerminal())

        patcher = mock.patch('snapcraft.internal.lifecycle.Popen',
                             new=mock.Mock(wraps=subprocess.Popen))
        self.popen_spy = patcher.start()
        self.addCleanup(patcher.stop)

    def make_snapcraft_yaml(self, n=1, snap_type='app', snapcraft_yaml=None):
        if not snapcraft_yaml:
            snapcraft_yaml = self.yaml_template.format(snap_type)
        super().make_snapcraft_yaml(snapcraft_yaml)
        self.state_dir = os.path.join(self.parts_dir, 'part1', 'state')


class SnapCommandTestCase(SnapCommandBaseTestCase):

    def test_snap_defaults(self):
        self.make_snapcraft_yaml()

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output,
                        Contains('\nSnapped snap-test_1.0_amd64.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

    def test_snap_fails_with_bad_type(self):
        self.make_snapcraft_yaml(snap_type='bad-type')

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftSchemaError,
            self.run_command, ['snap'])

        self.assertThat(str(raised), Contains(
            "bad-type' is not one of ['app', 'base', 'gadget', "
            "'kernel', 'os']"))

    def test_snap_is_the_default(self):
        self.make_snapcraft_yaml()

        result = self.run_command([])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output,
                        Contains('\nSnapped snap-test_1.0_amd64.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

    @mock.patch('os.getuid')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._inject_snapcraft')
    def test_snap_containerized(self,
                                mock_inject,
                                mock_container_run,
                                mock_getuid):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        mock_getuid.return_value = 1234
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))

        source = os.path.realpath(os.path.curdir)
        self.assertIn(
            'Mounting {} into container\n'.format(source),
            fake_logger.output)

        project_folder = '/root/build_snap-test'
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'init', 'ubuntu:xenial/amd64', fake_lxd.name]),
            call(['lxc', 'config', 'set', fake_lxd.name,
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'config', 'set', fake_lxd.name,
                  'raw.idmap', 'both {} 0'.format(mock_getuid.return_value)]),
            call(['lxc', 'start', fake_lxd.name]),
            call(['lxc', 'config', 'device', 'add', fake_lxd.name,
                  project_folder, 'disk', 'source={}'.format(source),
                  'path={}'.format(project_folder)]),
            call(['lxc', 'stop', '-f', fake_lxd.name]),
        ])
        mock_container_run.assert_has_calls([
            call(['python3', '-c', 'import urllib.request; ' +
                  'urllib.request.urlopen(' +
                  '"http://start.ubuntu.com/connectivity-check.html"' +
                  ', timeout=5)']),
            call(['apt-get', 'update']),
            call(['snapcraft', 'snap', '--output',
                  'snap-test_1.0_amd64.snap'], cwd=project_folder),
        ])

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('shutil.rmtree')
    @mock.patch('os.makedirs')
    @mock.patch('snapcraft.internal.lxd.Popen')
    @mock.patch('snapcraft.internal.lxd.open')
    def test_snap_containerized_remote(self,
                                       mock_open,
                                       mock_popen,
                                       mock_makedirs,
                                       mock_rmtree,
                                       mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        mock_open.return_value = mock.MagicMock(spec=open)
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_CONTAINER_BUILDS', 'myremote'))
        self.make_snapcraft_yaml()

        result = self.run_command(['--debug', 'snap'])

        self.assertThat(result.exit_code, Equals(0))

        source = os.path.realpath(os.path.curdir)
        self.assertIn(
            'Mounting {} into container\n'
            'Connecting to 127.0.0.1 via SSH\n'.format(source),
            fake_logger.output)

        project_folder = '/root/build_snap-test'
        tmpdir = os.path.expanduser(
            os.path.join('~', 'snap', 'lxd', 'common', 'snapcraft.tmp'))
        fake_lxd.check_output_mock.assert_has_calls([
            call(['ssh-keygen', '-o', '-N', '', '-f',
                 os.path.join(tmpdir, 'id_{}'.format(fake_lxd.name))]),
        ])
        fake_lxd.check_output_mock.assert_has_calls([
            call(['ssh', '-C', '-F', '/dev/null',
                  '-o', 'IdentityFile={}/id_{}'.format(tmpdir, fake_lxd.name),
                  '-o', 'StrictHostKeyChecking=no',
                  '-o', 'UserKnownHostsFile=/dev/null',
                  '-o', 'User=root',
                  '-p', '22', '127.0.0.1',
                  'ls']),
        ])
        mock_container_run.assert_has_calls([
            call(['mkdir', '-p', '/root/.ssh']),
            call(['chmod', '700', '/root/.ssh']),
            call(['tee', '-a', '/root/.ssh/authorized_keys'],
                 stdin=mock_open.return_value),
            call(['chmod', '600', '/root/.ssh/authorized_keys']),
            call(['apt-get', 'install', '-y', 'sshfs']),
        ])
        mock_popen.assert_has_calls([
            call(['/usr/lib/sftp-server'],
                 stdin=4, stdout=7),
            call(['ssh', '-C', '-F', '/dev/null',
                  '-o', 'IdentityFile={}/id_{}'.format(tmpdir, fake_lxd.name),
                  '-o', 'StrictHostKeyChecking=no',
                  '-o', 'UserKnownHostsFile=/dev/null',
                  '-o', 'User=root',
                  '-p', '22', '127.0.0.1',
                  'sshfs -o slave -o nonempty :{} {}'.format(
                      source, project_folder)],
                 stdin=6, stdout=5),
        ])

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('shutil.rmtree')
    @mock.patch('os.makedirs')
    @mock.patch('snapcraft.internal.lxd.Popen')
    @mock.patch('snapcraft.internal.lxd.open')
    def test_snap_containerized_remote_ssh_error(self,
                                                 mock_open,
                                                 mock_popen,
                                                 mock_makedirs,
                                                 mock_rmtree,
                                                 mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        mock_open.return_value = mock.MagicMock(spec=open)
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)

        def call_effect(*args, **kwargs):
            if args[0][:1] == ['ssh']:
                raise subprocess.CalledProcessError(
                    returncode=255, cmd=args[0])
            return fake_lxd.check_output_side_effect()(*args, **kwargs)

        fake_lxd.check_output_mock.side_effect = call_effect

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_CONTAINER_BUILDS', 'myremote'))
        self.make_snapcraft_yaml()

        self.assertIn('Failed to setup SSH',
                      str(self.assertRaises(
                          SnapcraftEnvironmentError,
                          self.run_command, ['--debug', 'snap'])))

        source = os.path.realpath(os.path.curdir)
        self.assertIn(
            'Mounting {} into container\n'
            'Connecting to 127.0.0.1 via SSH\n'.format(source),
            fake_logger.output)

        tmpdir = os.path.expanduser(
            os.path.join('~', 'snap', 'lxd', 'common', 'snapcraft.tmp'))
        fake_lxd.check_output_mock.assert_has_calls([
            call(['ssh-keygen', '-o', '-N', '', '-f',
                 os.path.join(tmpdir, 'id_{}'.format(fake_lxd.name))]),
        ])
        fake_lxd.check_output_mock.assert_has_calls([
            call(['ssh', '-C', '-F', '/dev/null',
                  '-o', 'IdentityFile={}/id_{}'.format(tmpdir, fake_lxd.name),
                  '-o', 'StrictHostKeyChecking=no',
                  '-o', 'UserKnownHostsFile=/dev/null',
                  '-o', 'User=root',
                  '-p', '22', '127.0.0.1',
                  'ls']),
        ])

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('snapcraft.internal.common.is_snap')
    def test_snap_containerized_inject_apt(self,
                                           mock_is_snap, mock_container_run):
        mock_is_snap.side_effect = lambda: False
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        self.run_command(['snap'])
        mock_container_run.assert_has_calls([
            call(['apt-get', 'install', 'snapcraft', '-y']),
        ])

    @mock.patch('snapcraft.internal.common.is_snap')
    @mock.patch('shutil.rmtree')
    @mock.patch('os.makedirs')
    def test_snap_containerized_inject_snap_socket_error(self,
                                                         mock_makedirs,
                                                         mock_rmtree,
                                                         mock_is_snap):
        mock_is_snap.side_effect = lambda: True
        fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.session_request_mock.side_effect = (
            requests.exceptions.ConnectionError(
                'Connection aborted.',
                FileNotFoundError(2, 'No such file or directory')))
        self.useFixture(fixture_setup.FakeLXD())
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        self.assertIn('Error connecting to ',
                      str(self.assertRaises(SnapcraftEnvironmentError,
                                            self.run_command, ['snap'])))
        # Temporary folder should remain in case of failure
        mock_rmtree.assert_not_called()

    @mock.patch('snapcraft.internal.common.is_snap')
    @mock.patch('shutil.rmtree')
    @mock.patch('shutil.copyfile')
    @mock.patch('os.makedirs')
    def test_snap_containerized_inject_snap_api_error(self,
                                                      mock_makedirs,
                                                      mock_copyfile,
                                                      mock_rmtree,
                                                      mock_is_snap):
        mock_is_snap.side_effect = lambda: True
        fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps = {}
        self.useFixture(fixture_setup.FakeLXD())
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        self.assertIn('Error querying \'core\' snap: not found',
                      str(self.assertRaises(SnapcraftEnvironmentError,
                                            self.run_command, ['snap'])))
        # Temporary folder should remain in case of failure
        mock_rmtree.assert_not_called()

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('snapcraft.internal.common.is_snap')
    @mock.patch('shutil.rmtree')
    @mock.patch('shutil.copyfile')
    @mock.patch('os.makedirs')
    def test_snap_containerized_inject_snap(self,
                                            mock_makedirs,
                                            mock_copyfile,
                                            mock_rmtree,
                                            mock_is_snap,
                                            mock_container_run):
        # Create open mock here for context manager to work correctly
        patcher = mock.patch('snapcraft.internal.lxd.open',
                             mock.mock_open())
        patcher.start()
        self.addCleanup(patcher.stop)

        mock_is_snap.side_effect = lambda: True
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        self.useFixture(fixture_setup.FakeSnapd())
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        self.run_command(['snap'])
        tmpdir = os.path.expanduser(
            os.path.join('~', 'snap', 'lxd', 'common', 'snapcraft.tmp'))
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'file', 'push',
                  os.path.join(tmpdir, 'core_123.assert'),
                  '{}/run/core_123.assert'.format(fake_lxd.name)]),
            call(['lxc', 'file', 'push',
                  os.path.join(tmpdir, 'core_123.snap'),
                  '{}/run/core_123.snap'.format(fake_lxd.name)]),
            call(['lxc', 'file', 'push',
                  os.path.join(tmpdir, 'snapcraft_345.assert'),
                  '{}/run/snapcraft_345.assert'.format(fake_lxd.name)]),
            call(['lxc', 'file', 'push',
                  os.path.join(tmpdir, 'snapcraft_345.snap'),
                  '{}/run/snapcraft_345.snap'.format(fake_lxd.name)]),
        ])
        mock_container_run.assert_has_calls([
            call(['apt-get', 'install', 'squashfuse', '-y']),
            call(['snap', 'ack', '/run/core_123.assert']),
            call(['snap', 'install', '/run/core_123.snap']),
            call(['snap', 'ack', '/run/snapcraft_345.assert']),
            call(['snap', 'install', '/run/snapcraft_345.snap', '--classic']),
        ])
        # Temporary folder should be removed in the end
        mock_rmtree.assert_has_calls([call(tmpdir)])

    @mock.patch('os.getuid')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('snapcraft.internal.common.is_snap')
    @mock.patch('shutil.rmtree')
    @mock.patch('shutil.copyfile')
    @mock.patch('os.makedirs')
    def test_snap_containerized_inject_snap_dangerous(self,
                                                      mock_makedirs,
                                                      mock_copyfile,
                                                      mock_rmtree,
                                                      mock_is_snap,
                                                      mock_container_run,
                                                      mock_getuid):
        # Create open mock here for context manager to work correctly
        patcher = mock.patch('snapcraft.internal.lxd.open',
                             mock.mock_open())
        patcher.start()
        self.addCleanup(patcher.stop)

        mock_is_snap.side_effect = lambda: True
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        mock_getuid.return_value = 1234
        fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps['snapcraft']['revision'] = 'x1'
        fake_snapd.snaps['snapcraft']['id'] = ''
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        self.run_command(['snap'])
        tmpdir = os.path.expanduser(
            os.path.join('~', 'snap', 'lxd', 'common', 'snapcraft.tmp'))
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'file', 'push',
                  os.path.join(tmpdir, 'core_123.assert'),
                  '{}/run/core_123.assert'.format(fake_lxd.name)]),
            call(['lxc', 'file', 'push',
                  os.path.join(tmpdir, 'core_123.snap'),
                  '{}/run/core_123.snap'.format(fake_lxd.name)]),
            call(['sudo', 'cp', '/var/lib/snapd/snaps/snapcraft_x1.snap',
                  os.path.join(tmpdir, 'snapcraft_x1.snap')]),
            call(['sudo', 'chown', str(os.getuid()),
                  os.path.join(tmpdir, 'snapcraft_x1.snap')]),
            call(['lxc', 'file', 'push',
                  os.path.join(tmpdir, 'snapcraft_x1.snap'),
                  '{}/run/snapcraft_x1.snap'.format(fake_lxd.name)]),
        ])
        mock_container_run.assert_has_calls([
            call(['apt-get', 'install', 'squashfuse', '-y']),
            call(['snap', 'ack', '/run/core_123.assert']),
            call(['snap', 'install', '/run/core_123.snap']),
            call(['snap', 'install', '/run/snapcraft_x1.snap',
                  '--dangerous', '--classic']),
        ])
        # Temporary folder should be removed in the end
        mock_rmtree.assert_has_calls([call(tmpdir)])

    @mock.patch('snapcraft.internal.lifecycle.ProgressBar')
    def test_snap_defaults_on_a_tty(self, progress_mock):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.useFixture(fixture_setup.FakeTerminal())

        self.make_snapcraft_yaml()

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output,
                        Contains('\nSnapped snap-test_1.0_amd64.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('snap-test_1.0_amd64.snap', FileExists())

    def test_snap_type_os_does_not_use_all_root(self):
        self.make_snapcraft_yaml(snap_type='os')

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output,
                        Contains('\nSnapped snap-test_1.0_amd64.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('snap-test_1.0_amd64.snap', FileExists())

    def test_snap_defaults_with_parts_in_prime(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        # Pretend this part has already been primed
        os.makedirs(self.state_dir)
        open(os.path.join(self.state_dir, 'prime'), 'w').close()

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            'Snapped snap-test_1.0_amd64.snap\n'))

        self.assertEqual(
            'Skipping pull part1 (already ran)\n'
            'Skipping build part1 (already ran)\n'
            'Skipping stage part1 (already ran)\n'
            'Skipping prime part1 (already ran)\n',
            fake_logger.output)

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('snap-test_1.0_amd64.snap', FileExists())

    def test_snap_from_dir(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        meta_dir = os.path.join('mysnap', 'meta')
        os.makedirs(meta_dir)
        with open(os.path.join(meta_dir, 'snap.yaml'), 'w') as f:
            f.write("""name: my_snap
version: 99
architectures: [amd64, armhf]
""")

        result = self.run_command(['snap', 'mysnap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            'Snapped my_snap_99_multi.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', os.path.abspath('mysnap'), 'my_snap_99_multi.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('my_snap_99_multi.snap', FileExists())

    def test_snap_from_dir_with_no_arch(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        meta_dir = os.path.join('mysnap', 'meta')
        os.makedirs(meta_dir)
        with open(os.path.join(meta_dir, 'snap.yaml'), 'w') as f:
            f.write("""name: my_snap
version: 99
""")

        result = self.run_command(['snap', 'mysnap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            'Snapped my_snap_99_all.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', os.path.abspath('mysnap'), 'my_snap_99_all.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('my_snap_99_all.snap', FileExists())

    def test_snap_from_dir_type_os_does_not_use_all_root(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        meta_dir = os.path.join('mysnap', 'meta')
        os.makedirs(meta_dir)
        with open(os.path.join(meta_dir, 'snap.yaml'), 'w') as f:
            f.write("""name: my_snap
version: 99
architectures: [amd64, armhf]
type: os
""")
        self.make_snapcraft_yaml()

        result = self.run_command(['snap', 'mysnap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            'Snapped my_snap_99_multi.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', os.path.abspath('mysnap'), 'my_snap_99_multi.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('my_snap_99_multi.snap', FileExists())

    def test_snap_with_output(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        result = self.run_command(['snap', '--output', 'mysnap.snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            'Snapped mysnap.snap\n'))

        self.assertThat(fake_logger.output, Equals(
            'Preparing to pull part1 \n'
            'Pulling part1 \n'
            'Preparing to build part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Priming part1 \n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'mysnap.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('mysnap.snap', FileExists())

    def test_load_config_with_invalid_plugin_exits_with_error(self):
        self.make_snapcraft_yaml(snapcraft_yaml=dedent("""\
            name: test-package
            version: 1
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: does-not-exist
        """))

        raised = self.assertRaises(
            snapcraft.internal.errors.PluginError,
            self.run_command, ['snap'])

        self.assertThat(str(raised), Equals(
            "Issue while loading part: unknown plugin: 'does-not-exist'"))

    @mock.patch('time.time')
    def test_snap_renames_stale_snap_build(self, mocked_time):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        mocked_time.return_value = 1234

        snap_build = 'snap-test_1.0_amd64.snap-build'
        with open(snap_build, 'w') as fd:
            fd.write('signed assertion?')

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            'Snapped snap-test_1.0_amd64.snap\n'))

        snap_build_renamed = snap_build + '.1234'
        self.assertEqual([
            'Preparing to pull part1 ',
            'Pulling part1 ',
            'Preparing to build part1 ',
            'Building part1 ',
            'Staging part1 ',
            'Priming part1 ',
            'Renaming stale build assertion to {}'.format(snap_build_renamed),
            ], fake_logger.output.splitlines())

        self.assertThat('snap-test_1.0_amd64.snap', FileExists())
        self.assertThat(snap_build, Not(FileExists()))
        self.assertThat(snap_build_renamed, FileExists())
        self.assertThat(
            snap_build_renamed, FileContains('signed assertion?'))


class SnapCommandAsDefaultTestCase(SnapCommandBaseTestCase):

    scenarios = [
        ('no parallel builds', dict(options=['--no-parallel-builds'])),
        ('target architecture', dict(options=['--target-arch', 'i386'])),
        ('geo ip', dict(options=['--enable-geoip'])),
        ('all', dict(options=['--no-parallel-builds', '--target-arch=i386',
                              '--enable-geoip']))
    ]

    def test_snap_defaults(self):
        """The arguments should not be rejected when 'snap' is implicit."""
        self.make_snapcraft_yaml()

        result = self.run_command(self.options)

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output,
                        Contains('\nSnapped snap-test_1.0'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
