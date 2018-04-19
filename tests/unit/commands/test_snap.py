# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
from tests import fixture_setup
from . import CommandBaseTestCase


class SnapCommandBaseTestCase(CommandBaseTestCase):

    yaml_template = dedent("""\
        name: snap-test
        version: 1.0
        summary: test snapping
        description: if snap is successful a snap package will be available
        architectures:
          - build-on: all
            run-on: 'amd64'
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

        patcher = mock.patch('snapcraft.internal.lifecycle._packer.Popen',
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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments',
            '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

    @mock.patch('snapcraft.internal.repo.check_for_command')
    @mock.patch('snapcraft.internal.lifecycle._packer._run_mksquashfs')
    def test_mksquashfs_from_snap_used_if_using_snap(self, mock_run_mksquashfs,
                                                     mock_check_command):
        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        self.make_snapcraft_yaml()

        original_exists = os.path.exists

        def _fake_exists(path):
            if path == '/snap/snapcraft/current/usr/bin/mksquashfs':
                return True
            else:
                return original_exists(path)

        with mock.patch('os.path.exists', side_effect=_fake_exists):
            self.run_command(['snap'])

        mksquashfs_path = os.path.join('/snap', 'snapcraft', 'current',
                                       'usr', 'bin', 'mksquashfs')

        mock_run_mksquashfs.assert_called_once_with(
            mksquashfs_path, directory=self.prime_dir, snap_name='snap-test',
            snap_type='app', output_snap_name='snap-test_1.0_amd64.snap')

    @mock.patch('snapcraft.internal.common.is_docker_instance')
    @mock.patch('snapcraft.internal.repo.check_for_command')
    @mock.patch('snapcraft.internal.lifecycle._packer._run_mksquashfs')
    def test_mksquashfs_from_snap_used_if_docker(self, mock_run_mksquashfs,
                                                 mock_check_command,
                                                 mock_is_docker):
        mock_is_docker.return_value = True
        self.make_snapcraft_yaml()

        original_exists = os.path.exists

        def _fake_exists(path):
            if path == '/snap/snapcraft/current/usr/bin/mksquashfs':
                return True
            else:
                return original_exists(path)

        with mock.patch('os.path.exists', side_effect=_fake_exists):
            self.run_command(['snap'])

        mksquashfs_path = os.path.join('/snap', 'snapcraft', 'current',
                                       'usr', 'bin', 'mksquashfs')

        mock_run_mksquashfs.assert_called_once_with(
            mksquashfs_path, directory=self.prime_dir, snap_name='snap-test',
            snap_type='app', output_snap_name='snap-test_1.0_amd64.snap')

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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments',
            '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

    def test_snap_containerized_remote_fails(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_CONTAINER_BUILDS', 'myremote'))
        self.make_snapcraft_yaml()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command, ['snap'])

        self.assertThat(str(raised), Contains(
            'The experimental feature of using non-local LXD remotes '
            'with SNAPCRAFT_CONTAINER_BUILDS has been dropped.'))

    def test_use_of_both_build_env_affecting_vars_fails(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_CONTAINER_BUILDS', 'on'))
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_ENVIRONMENT', 'host'))
        self.make_snapcraft_yaml()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command, ['snap'])

        self.assertThat(str(raised), Contains(
            'SNAPCRAFT_BUILD_ENVIRONMENT and SNAPCRAFT_CONTAINER_BUILDS '
            'cannot be used together.\n'
            'Given that SNAPCRAFT_BUILD_ENVIRONMENT is deprecated, '
            'unset that variable from the environment and try again.'))

    def test_snap_type_os_does_not_use_all_root(self):
        self.make_snapcraft_yaml(snap_type='os')

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output,
                        Contains('\nSnapped snap-test_1.0_amd64.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('snap-test_1.0_amd64.snap', FileExists())

    def test_snap_type_base_does_not_use_all_root(self):
        self.make_snapcraft_yaml(snap_type='base')

        result = self.run_command(['snap'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output,
                        Contains('\nSnapped snap-test_1.0_amd64.snap\n'))

        self.popen_spy.assert_called_once_with([
            'mksquashfs', self.prime_dir, 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments'],
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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments',
            '-all-root'],
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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments',
            '-all-root'],
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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments',
            '-all-root'],
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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments'],
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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments',
            '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)

        self.assertThat('mysnap.snap', FileExists())

    def test_load_config_with_invalid_plugin_raises_exception(self):
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

        self.assertThat(
            raised.message, Equals("unknown plugin: 'does-not-exist'"))

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


class SnapCommandWithContainerBuildTestCase(SnapCommandBaseTestCase):

    scenarios = (
        ('with SUDO_UID', {
            'SUDO_UID': 'test_sudo_uid',
            'getuid': None,
            'expected_idmap': 'test_sudo_uid'}),
        ('without SUDO_UID', {
            'SUDO_UID': None,
            'getuid': 'test_getuid',
            'expected_idmap': 'test_getuid'}),
    )

    @mock.patch('os.getuid')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._inject_snapcraft')
    def test_snap_containerized(self,
                                mock_inject,
                                mock_container_run,
                                mock_getuid):
        self.useFixture(
            fixtures.EnvironmentVariable('SUDO_UID', self.SUDO_UID))
        mock_getuid.return_value = self.getuid
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
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
        self.assertThat(fake_logger.output, Contains(
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Mounting {} into container\n'.format(source)))

        container_name = 'local:snapcraft-snap-test'
        project_folder = '/root/build_snap-test'
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'init', 'ubuntu:xenial', container_name]),
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.LC_ALL', 'C.UTF-8']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_IMAGE_INFO',
                  '{"fingerprint": "test-fingerprint", '
                  '"architecture": "test-architecture", '
                  '"created_at": "test-created-at"}']),
            call(['lxc', 'config', 'set', container_name,
                  'raw.idmap', 'both {} {}'.format(
                    self.expected_idmap, '0')]),
            call(['lxc', 'config', 'device', 'add', container_name,
                  'fuse', 'unix-char', 'path=/dev/fuse']),
            call(['lxc', 'start', container_name]),
            call(['lxc', 'config', 'device', 'add', container_name,
                  project_folder, 'disk', 'source={}'.format(source),
                  'path={}'.format(project_folder)]),
            call(['lxc', 'stop', '-f', container_name]),
        ])
        mock_container_run.assert_has_calls([
            call(['python3', '-c', mock.ANY]),
            call(['apt-get', 'update']),
            call(['apt-get', 'install', 'squashfuse', '-y']),
            call(['snapcraft', 'snap', '--output',
                  'snap-test_1.0_amd64.snap'],
                 cwd=project_folder, user='root'),
        ])

    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('os.getuid')
    def test_snap_containerized_exists_running(self,
                                               mock_getuid,
                                               mock_container_run):
        self.useFixture(
            fixtures.EnvironmentVariable('SUDO_UID', self.SUDO_UID))
        mock_getuid.return_value = self.getuid
        fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)
        # Container was created before and is running
        fake_lxd.name = 'local:snapcraft-snap-test'
        fake_lxd.status = 'Running'
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_CONTAINER_BUILDS', '1'))
        self.make_snapcraft_yaml()

        self.run_command(['snap'])

        source = os.path.realpath(os.path.curdir)
        project_folder = '/root/build_snap-test'
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'config', 'device', 'add', fake_lxd.name,
                  project_folder, 'disk', 'source={}'.format(source),
                  'path={}'.format(project_folder)]),
            call(['lxc', 'stop', '-f', fake_lxd.name]),
        ])
        mock_container_run.assert_has_calls([
            call(['python3', '-c', mock.ANY]),
            call(['snapcraft', 'snap', '--output',
                  'snap-test_1.0_amd64.snap'],
                 cwd=project_folder, user='root'),
        ])

    @mock.patch('os.getuid')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._container_run')
    @mock.patch('snapcraft.internal.lxd.Containerbuild._inject_snapcraft')
    def test_snap_containerized_exists_stopped(self,
                                               mock_inject,
                                               mock_container_run,
                                               mock_getuid):

        self.useFixture(
            fixtures.EnvironmentVariable('SUDO_UID', self.SUDO_UID))
        mock_getuid.return_value = self.getuid
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd
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

        self.assertIn(
            'Waiting for a network connection...\n'
            'Network connection established\n',
            fake_logger.output)

        container_name = 'local:snapcraft-snap-test'
        project_folder = '/root/build_snap-test'
        fake_lxd.check_call_mock.assert_has_calls([
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_SETUP_CORE', '1']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.LC_ALL', 'C.UTF-8']),
            call(['lxc', 'config', 'set', container_name,
                  'environment.SNAPCRAFT_IMAGE_INFO',
                  '{"fingerprint": "test-fingerprint", '
                  '"architecture": "test-architecture", '
                  '"created_at": "test-created-at"}']),
            call(['lxc', 'config', 'set', container_name,
                  'raw.idmap', 'both {} {}'.format(
                    self.expected_idmap, 0)]),
            call(['lxc', 'config', 'device', 'remove', container_name,
                  project_folder]),
            call(['lxc', 'config', 'device', 'add', container_name,
                  'fuse', 'unix-char', 'path=/dev/fuse']),
            call(['lxc', 'start', container_name]),
            call(['lxc', 'stop', '-f', container_name]),
        ])
        mock_container_run.assert_has_calls([
            call(['python3', '-c', mock.ANY]),
            call(['snapcraft', 'snap', '--output',
                  'snap-test_1.0_amd64.snap'],
                 cwd=project_folder, user='root'),
        ])
        # Ensure there's no unexpected calls eg. two network checks
        self.assertThat(mock_container_run.call_count, Equals(2))


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
            '-noappend', '-comp', 'xz', '-no-xattrs', '-no-fragments',
            '-all-root'],
            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
