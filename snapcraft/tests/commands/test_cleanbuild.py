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

import logging
import os
import tarfile
from testtools.matchers import Contains
from unittest import mock

import fixtures

from snapcraft.main import main
from snapcraft import tests
from snapcraft.tests.test_lxd import check_output_side_effect


class CleanBuildCommandTestCase(tests.TestCase):

    yaml_template = """name: snap-test
version: 1.0
summary: test cleanbuild
description: if snap is succesful a snap package will be available
architectures: ['amd64']
confinement: strict
grade: stable

parts:
    part1:
      plugin: nil
"""

    def setUp(self):
        super().setUp()
        patcher = mock.patch('snapcraft.internal.lxd.check_call')
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.internal.lxd.check_output')
        self.check_output_mock = patcher.start()
        self.check_output_mock.side_effect = check_output_side_effect()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.internal.lxd.sleep', lambda _: None)
        patcher.start()
        self.addCleanup(patcher.stop)

    def make_snapcraft_yaml(self, n=1):
        super().make_snapcraft_yaml(self.yaml_template)
        self.state_dir = os.path.join(self.parts_dir, 'part1', 'state')

    def test_cleanbuild(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml()
        # simulate build artifacts

        dirs = [
            os.path.join(self.parts_dir, 'part1', 'src'),
            self.stage_dir,
            self.prime_dir,
            os.path.join(self.parts_dir, 'plugins'),
        ]
        files_tar = [
            os.path.join(self.parts_dir, 'plugins', 'x-plugin.py'),
            'main.c',
        ]
        files_no_tar = [
            os.path.join(self.stage_dir, 'binary'),
            os.path.join(self.prime_dir, 'binary'),
            'snap-test.snap',
            'snap-test_1.0_source.tar.bz2',
        ]
        for d in dirs:
            os.makedirs(d)
        for f in files_tar + files_no_tar:
            open(f, 'w').close()

        main(['cleanbuild', '--debug'])

        self.assertIn(
            'Setting up container with project assets\n'
            'Copying snapcraft cache into container\n'
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Retrieved snap-test_1.0_amd64.snap\n',
            fake_logger.output)

        with tarfile.open('snap-test_1.0_source.tar.bz2') as tar:
            tar_members = tar.getnames()

        for f in files_no_tar:
            f = os.path.relpath(f)
            self.assertFalse('./{}'.format(f) in tar_members,
                             '{} should not be in {}'.format(f, tar_members))
        for f in files_tar:
            f = os.path.relpath(f)
            self.assertTrue('./{}'.format(f) in tar_members,
                            '{} should be in {}'.format(f, tar_members))

        # Also assert that the snapcraft.yaml made it into the cleanbuild tar
        self.assertThat(
            tar_members,
            Contains(os.path.join('.', 'snap', 'snapcraft.yaml')),
            'snap/snapcraft unexpectedly excluded from tarball')

    def test_no_lxd(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml()

        self.check_output_mock.side_effect = check_output_side_effect(
            fail_on_default=True)

        raised = self.assertRaises(
            SystemExit,
            main, ['cleanbuild'])

        self.maxDiff = None
        self.assertEqual(1, raised.code)
        self.assertEqual(
            fake_logger.output,
            'The lxd package is not installed, in order to use `cleanbuild` '
            'you must install lxd onto your system. Refer to the '
            '"Ubuntu Desktop and Ubuntu Server" section on '
            'https://linuxcontainers.org/lxd/getting-started-cli/'
            '#ubuntu-desktop-and-ubuntu-server to enable a proper setup.\n')
