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
import subprocess
import tarfile
from textwrap import dedent

import fixtures
from testtools.matchers import Contains, Equals

from snapcraft import tests
from . import CommandBaseTestCase

from snapcraft.internal.errors import InvalidContainerRemoteError


class CleanBuildCommandBaseTestCase(CommandBaseTestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        self.make_snapcraft_yaml(dedent("""\
            name: snap-test
            version: 1.0
            summary: test cleanbuild
            description: if snap is succesful a snap package will be available
            architectures: ['amd64']
            confinement: strict
            grade: stable

            parts:
                part1:
                  plugin: nil
        """))
        self.state_dir = os.path.join(self.parts_dir, 'part1', 'state')

    def test_cleanbuild(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.useFixture(tests.fixture_setup.FakeLXD())


class CleanBuildCommandTestCase(CleanBuildCommandBaseTestCase):

    def setUp(self):
        super().setUp()

        # simulate build artifacts
        dirs = [
            os.path.join(self.parts_dir, 'part1', 'src'),
            self.stage_dir,
            self.prime_dir,
            os.path.join(self.parts_dir, 'plugins'),
        ]
        self.files_tar = [
            os.path.join(self.parts_dir, 'plugins', 'x-plugin.py'),
            'main.c',
        ]
        self.files_no_tar = [
            os.path.join(self.stage_dir, 'binary'),
            os.path.join(self.prime_dir, 'binary'),
            'snap-test.snap',
            'snap-test_1.0_source.tar.bz2',
        ]
        for d in dirs:
            os.makedirs(d)
        for f in self.files_tar + self.files_no_tar:
            open(f, 'w').close()

    def test_cleanbuild(self):
        self.useFixture(tests.fixture_setup.FakeLXD())

        result = self.run_command(['cleanbuild'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertIn(
            'Setting up container with project assets\n'
            'Retrieved snap-test_1.0_amd64.snap\n',
            self.fake_logger.output)

        with tarfile.open('snap-test_1.0_source.tar.bz2') as tar:
            tar_members = tar.getnames()

        for f in self.files_no_tar:
            f = os.path.relpath(f)
            self.assertFalse('./{}'.format(f) in tar_members,
                             '{} should not be in {}'.format(f, tar_members))
        for f in self.files_tar:
            f = os.path.relpath(f)
            self.assertTrue('./{}'.format(f) in tar_members,
                            '{} should be in {}'.format(f, tar_members))

        # Also assert that the snapcraft.yaml made it into the cleanbuild tar
        self.assertThat(
            tar_members,
            Contains(os.path.join('.', 'snap', 'snapcraft.yaml')),
            'snap/snapcraft unexpectedly excluded from tarball')

    def test_cleanbuild_debug_appended_goes_to_shell_on_errors(self):
        fake_lxd = tests.fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)

        def call_effect(*args, **kwargs):
            # Fail on an actual snapcraft command and not the command
            # for the installation of it.
            if 'snapcraft snap' in ' '.join(args[0]):
                raise subprocess.CalledProcessError(
                    returncode=255, cmd=args[0])

        fake_lxd.check_call_mock.side_effect = call_effect

        result = self.run_command(['cleanbuild', '--debug'])
        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.fake_logger.output, Contains(
            'Debug mode enabled, dropping into a shell'))

    def test_cleanbuild_debug_prepended_goes_to_shell_on_errors(self):
        fake_lxd = tests.fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)

        def call_effect(*args, **kwargs):
            # Fail on an actual snapcraft command and not the command
            # for the installation of it.
            if 'snapcraft snap' in ' '.join(args[0]):
                raise subprocess.CalledProcessError(
                    returncode=255, cmd=args[0])

        fake_lxd.check_call_mock.side_effect = call_effect

        result = self.run_command(['--debug', 'cleanbuild'])
        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.fake_logger.output, Contains(
            'Debug mode enabled, dropping into a shell'))

    def test_invalid_remote(self):
        fake_lxd = tests.fixture_setup.FakeLXD()
        self.useFixture(fake_lxd)

        self.assertIn(
            "'foo/bar' is not a valid LXD remote name",
            str(self.assertRaises(
                InvalidContainerRemoteError,
                self.run_command, ['cleanbuild', '--remote', 'foo/bar'])))
