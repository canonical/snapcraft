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
import os.path
from unittest import mock

import fixtures

from snapcraft import (
    common,
    tests,
)
from snapcraft.commands import upload


class UploadCommandTestCase(tests.TestCase):

    yaml_template = """name: snap-test
version: 1.0
summary: test strip
description: if snap is succesful a snap package will be available
architectures: ['amd64']

parts:
    part1:
      plugin: nil
"""

    def setUp(self):
        super().setUp()
        patcher = mock.patch('snapcraft.commands.upload.upload')
        self.mock_upload = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.commands.upload.os')
        self.mock_os = patcher.start()
        self.mock_os.path.exists.return_value = False
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.commands.upload.load_config')
        self.mock_load_config = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.commands.snap.subprocess.check_call')
        patcher.start()
        self.addCleanup(patcher.stop)

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        self.make_snapcraft_yaml()

    def make_snapcraft_yaml(self, n=1):
        super().make_snapcraft_yaml(self.yaml_template)
        self.state_file = os.path.join(common.get_partsdir(), 'part1', 'state')

    def test_upload(self):
        upload.main()

        self.assertEqual(
            'Snap snap-test_1.0_amd64.snap not found. '
            'Running snap step to create it.\n'
            'Pulling part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Stripping part1 \n'
            'Snapping snap-test_1.0_amd64.snap\n'
            'Snapped snap-test_1.0_amd64.snap\n',
            self.fake_logger.output)

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')
        self.assertTrue(self.state_file,
                        'Expected a state file for the part1 part')

        with open(self.state_file) as sf:
            state = sf.readlines()
        self.assertEqual(len(state), 1, 'Expected only one line in the state '
                         'file for the part1 part')
        self.assertEqual(state[0], 'strip', "Expected the state file for "
                         "part1 to be 'strip'")

        self.mock_upload.assert_called_once_with(
            'snap-test_1.0_amd64.snap',
            'snap-test',
            config=self.mock_load_config.return_value)

    def test_upload_failed(self):
        self.mock_upload.return_value = False

        upload.main()

        self.assertEqual(
            'Snap snap-test_1.0_amd64.snap not found. '
            'Running snap step to create it.\n'
            'Pulling part1 \n'
            'Building part1 \n'
            'Staging part1 \n'
            'Stripping part1 \n'
            'Snapping snap-test_1.0_amd64.snap\n'
            'Snapped snap-test_1.0_amd64.snap\n',
            self.fake_logger.output)

        self.assertTrue(os.path.exists(common.get_stagedir()),
                        'Expected a stage directory')
        self.assertTrue(self.state_file,
                        'Expected a state file for the part1 part')

        with open(self.state_file) as sf:
            state = sf.readlines()
        self.assertEqual(len(state), 1, 'Expected only one line in the state '
                         'file for the part1 part')
        self.assertEqual(state[0], 'strip', "Expected the state file for "
                         "part1 to be 'strip'")

        self.mock_upload.assert_called_once_with(
            'snap-test_1.0_amd64.snap',
            'snap-test',
            config=self.mock_load_config.return_value)

    def test_just_upload_if_snap_file_exists(self):
        self.mock_os.path.exists.return_value = True

        upload.main()

        # stages are not build if snap file already exists
        self.assertFalse(os.path.exists(common.get_stagedir()),
                         'Expected a stage directory')
        self.assertFalse(os.path.exists(self.state_file))

        self.mock_upload.assert_called_once_with(
            'snap-test_1.0_amd64.snap',
            'snap-test',
            config=self.mock_load_config.return_value)
