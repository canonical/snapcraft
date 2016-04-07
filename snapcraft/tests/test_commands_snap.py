# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
from unittest import mock

import fixtures

from snapcraft.main import main
from snapcraft import (
    common,
    tests,
)


class SnapCommandTestCase(tests.TestCase):

    yaml_template = """name: snap-test
version: 1.0
summary: test strip
description: if snap is succesful a snap package will be available
architectures: ['amd64']
type: {}

parts:
    part1:
      plugin: nil
"""

    def make_snapcraft_yaml(self, n=1, snap_type='app'):
        snapcraft_yaml = self.yaml_template.format(snap_type)
        super().make_snapcraft_yaml(snapcraft_yaml)
        self.state_dir = os.path.join(common.get_partsdir(), 'part1', 'state')

    @mock.patch('subprocess.check_call')
    def test_snap_defaults(self, mock_call):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        main(['snap'])

        self.assertEqual(
            'Preparing to pull part1 \n'
            'Pulling part1 \n'
            'Preparing to build part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Stripping part1 \n'
            'Snapping snap-test_1.0_amd64.snap\n'
            'Snapped snap-test_1.0_amd64.snap\n',
            fake_logger.output)

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')

        self.verify_state('part1', self.state_dir, 'strip')

        mock_call.assert_called_once_with([
            'mksquashfs', common.get_snapdir(), 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'])

    @mock.patch('subprocess.check_call')
    def test_snap_type_os_does_not_use_all_root(self, mock_call):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml(snap_type='os')

        main(['snap'])

        self.assertEqual(
            'Preparing to pull part1 \n'
            'Pulling part1 \n'
            'Preparing to build part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Stripping part1 \n'
            'Snapping snap-test_1.0_amd64.snap\n'
            'Snapped snap-test_1.0_amd64.snap\n',
            fake_logger.output)

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')

        self.verify_state('part1', self.state_dir, 'strip')

        mock_call.assert_called_once_with([
            'mksquashfs', common.get_snapdir(), 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs'])

    @mock.patch('subprocess.check_call')
    def test_snap_defaults_with_parts_in_strip(self, mock_call):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        # Pretend this part has already been stripped
        os.makedirs(self.state_dir)
        open(os.path.join(self.state_dir, 'strip'), 'w').close()

        main(['snap'])

        self.assertEqual(
            'Skipping pull part1 (already ran)\n'
            'Skipping build part1 (already ran)\n'
            'Skipping stage part1 (already ran)\n'
            'Skipping strip part1 (already ran)\n'
            'Snapping snap-test_1.0_amd64.snap\n'
            'Snapped snap-test_1.0_amd64.snap\n',
            fake_logger.output)

        mock_call.assert_called_once_with([
            'mksquashfs', common.get_snapdir(), 'snap-test_1.0_amd64.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'])

    @mock.patch('subprocess.check_call')
    def test_snap_from_dir(self, mock_call):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        meta_dir = os.path.join('mysnap', 'meta')
        os.makedirs(meta_dir)
        with open(os.path.join(meta_dir, 'snap.yaml'), 'w') as f:
            f.write("""name: my_snap
version: 99
architectures: [amd64, armhf]
""")

        main(['snap', 'mysnap'])

        self.assertEqual(
            'Snapping my_snap_99_multi.snap\n'
            'Snapped my_snap_99_multi.snap\n',
            fake_logger.output)

        mock_call.assert_called_once_with([
            'mksquashfs', os.path.abspath('mysnap'), 'my_snap_99_multi.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'])

    @mock.patch('subprocess.check_call')
    def test_snap_from_dir_with_no_arch(self, mock_call):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        meta_dir = os.path.join('mysnap', 'meta')
        os.makedirs(meta_dir)
        with open(os.path.join(meta_dir, 'snap.yaml'), 'w') as f:
            f.write("""name: my_snap
version: 99
""")

        main(['snap', 'mysnap'])

        self.assertEqual(
            'Snapping my_snap_99_all.snap\n'
            'Snapped my_snap_99_all.snap\n',
            fake_logger.output)

        mock_call.assert_called_once_with([
            'mksquashfs', os.path.abspath('mysnap'), 'my_snap_99_all.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'])

    @mock.patch('subprocess.check_call')
    def test_snap_from_dir_type_os_does_not_use_all_root(self, mock_call):
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

        main(['snap', 'mysnap'])

        self.assertEqual(
            'Snapping my_snap_99_multi.snap\n'
            'Snapped my_snap_99_multi.snap\n',
            fake_logger.output)

        mock_call.assert_called_once_with([
            'mksquashfs', os.path.abspath('mysnap'), 'my_snap_99_multi.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs'])

    @mock.patch('subprocess.check_call')
    def test_snap_with_output(self, mock_call):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()

        main(['snap', '--output', 'mysnap.snap'])

        self.assertEqual(
            'Preparing to pull part1 \n'
            'Pulling part1 \n'
            'Preparing to build part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Stripping part1 \n'
            'Snapping mysnap.snap\n'
            'Snapped mysnap.snap\n',
            fake_logger.output)

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')

        self.verify_state('part1', self.state_dir, 'strip')

        mock_call.assert_called_once_with([
            'mksquashfs', common.get_snapdir(), 'mysnap.snap',
            '-noappend', '-comp', 'xz', '-no-xattrs', '-all-root'])
