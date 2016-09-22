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

import os
import logging
from unittest import mock

import fixtures
import tempfile
import shutil

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests,
)


class SignBuildTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft._store._get_data_from_snap_file')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_saved(
            self, mock_installed, mock_get_snap_data, mock_check_output,
            mock_get_account_info):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'snaps': {
                '16': {
                    'test-snap': {'snap-id': 'snap-id'},
                }
            }
        }
        mock_get_snap_data.return_value = {
            'name': 'test-snap',
            'grade': 'stable',
        }
        mock_check_output.return_value = 'Mocked assertion'

        with tempfile.TemporaryDirectory() as temp_dir:
            # Copy testing snap.
            test_snap = os.path.join(
                os.path.dirname(tests.__file__), 'data', 'test-snap.snap')
            snap_path = os.path.join(temp_dir, 'test-snap.snap')
            shutil.copyfile(test_snap, snap_path)

            main(['sign-build', snap_path, '--local'])

            # Trace snap-build assertion file.
            snap_build_path = os.path.join(temp_dir, 'test-snap.snap-build')
            with open(snap_build_path) as snap_build:
                self.assertEqual('Mocked assertion', snap_build.read())

            self.assertEqual([
                'Build assertion {} saved to disk.'.format(snap_build_path),
            ], self.fake_logger.output.splitlines())
