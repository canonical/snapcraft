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
from unittest import mock
from unittest.mock import call
import snapcraft.internal.errors
import snapcraft.internal.project_loader.errors

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
            snapcraft.internal.project_loader.errors.YamlValidationError,
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
            'Using default LXD remote because '
            'SNAPCRAFT_CONTAINER_BUILDS is set to 1\n'
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Mounting {} into container\n'.format(source),
            fake_logger.output)

        container_name = 'local:snapcraft-snap-test'
        project_folder = '/root/build_snap-test'
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'init', 'ubuntu:xenial/amd64', container_name]),
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.LC_ALL', 'C.UTF-8']),
            call(['lxc', 'config', 'set', container_name,
                  'raw.idmap', 'both {} 0'.format(mock_getuid.return_value)]),
            call(['lxc', 'config', 'device', 'add', container_name,
                  'fuse', 'unix-char', 'path=/dev/fuse']),
            call(['lxc', 'start', container_name]),
            call(['lxc', 'config', 'device', 'add', container_name,
                  project_folder, 'disk', 'source={}'.format(source),
                  'path={}'.format(project_folder)]),
            call(['lxc', 'stop', '-f', container_name]),
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
    @mock.patch('os.pipe')
    @mock.patch('snapcraft.internal.lxd.open')
    def test_snap_containerized_remote(self,
                                       mock_open,
                                       mock_pipe,
                                       mock_makedirs,
                                       mock_rmtree,
                                       mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        mock_open.return_value = mock.MagicMock(spec=open)
        mock_pipe.return_value = (9, 9)
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
            "Using LXD remote 'myremote' from SNAPCRAFT_CONTAINER_BUILDS\n"
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Mounting {} into container\n'.format(source),
            fake_logger.output)

        project_folder = '/root/build_snap-test'
        mock_container_run.assert_has_calls([
            call(['apt-get', 'install', '-y', 'sshfs']),
        ])
        fake_lxd.popen_mock.assert_has_calls([
            call(['/usr/lib/sftp-server'],
                 stdin=9, stdout=9),
            call(['lxc', 'exec', fake_lxd.name, '--',
                  'sshfs', '-o', 'slave', '-o', 'nonempty',
                  ':{}'.format(source), project_folder],
                 stdin=9, stdout=9),
        ])

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('shutil.rmtree')
    @mock.patch('os.makedirs')
    @mock.patch('snapcraft.internal.lxd.open')
    def test_snap_containerized_invalid_remote(self,
                                               mock_open,
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
            'SNAPCRAFT_CONTAINER_BUILDS', 'foo/bar'))
        self.make_snapcraft_yaml()

        self.assertIn(
            "'foo/bar' is not a valid LXD remote name",
            str(self.assertRaises(
                snapcraft.internal.errors.InvalidContainerRemoteError,
                self.run_command, ['--debug', 'snap'])))

    @mock.patch('os.getuid')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._inject_snapcraft')
    def test_snap_containerized_exists_stopped(self,
                                               mock_inject,
                                               mock_container_run,
                                               mock_getuid):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
        mock_getuid.return_value = 1234
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        # Container was created before, and isn't running
        fake_lxd.devices = '{"/root/build_snap-test":[]}'
        fake_lxd.name = 'local:snapcraft-snap-test'
        fake_lxd.status = 'Stopped'
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))

        source = os.path.realpath(os.path.curdir)
        self.assertIn(
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Mounting {} into container\n'.format(source),
            fake_logger.output)

        container_name = 'local:snapcraft-snap-test'
        project_folder = '/root/build_snap-test'
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.LC_ALL', 'C.UTF-8']),
            call(['lxc', 'config', 'set', container_name,
                  'raw.idmap', 'both {} 0'.format(mock_getuid.return_value)]),
            call(['lxc', 'config', 'device', 'remove', container_name,
                  project_folder]),
            call(['lxc', 'config', 'device', 'add', container_name,
                  'fuse', 'unix-char', 'path=/dev/fuse']),
            call(['lxc', 'start', container_name]),
            call(['lxc', 'stop', '-f', container_name]),
        ])
        mock_container_run.assert_has_calls([
              call(['python3', '-c', 'import urllib.request; ' +
                    'urllib.request.urlopen(' +
                    '"http://start.ubuntu.com/connectivity-check.html"' +
                    ', timeout=5)']),
              call(['apt-get', 'update']),
              call(['snapcraft', 'snap', '--output',
                    'snap-test_1.0_amd64.snap'],
                   cwd=project_folder),
        ])

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

        self.assertThat(
            fake_logger.output,
            Equals(
                'Skipping pull part1 (already ran)\n'
                'Skipping build part1 (already ran)\n'
                'Skipping stage part1 (already ran)\n'
                'Skipping prime part1 (already ran)\n'))

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
            'mksquashfs', 'mysnap', 'my_snap_99_multi.snap',
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
            'mksquashfs', 'mysnap', 'my_snap_99_all.snap',
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
            'mksquashfs', 'mysnap', 'my_snap_99_multi.snap',
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
        self.assertThat(
            fake_logger.output.splitlines(),
            Equals([
                'Preparing to pull part1 ',
                'Pulling part1 ',
                'Preparing to build part1 ',
                'Building part1 ',
                'Staging part1 ',
                'Priming part1 ',
                'Renaming stale build assertion to {}'.format(
                    snap_build_renamed),
            ]))

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
