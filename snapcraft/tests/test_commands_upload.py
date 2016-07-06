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

import glob
import logging
import os
import os.path
from unittest import mock

import docopt
import fixtures

import snapcraft._store
from snapcraft import (
    storeapi,
    tests
)
from snapcraft.main import main


class UploadCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    def test_upload_without_snap_must_raise_exception(self):
        with self.assertRaises(docopt.DocoptExit) as raised:
            main(['upload'])

        self.assertTrue('Usage:' in str(raised.exception))

    def test_upload_snap(self):
        mock_tracker = mock.Mock(storeapi.StatusTracker)
        mock_tracker.track.return_value = {
            'code': 'ready_to_release',
            'processed': True,
            'can_release': True,
            'url': '/fake/url',
            'revision': 1,
        }
        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = mock_tracker

        # Avoiding a io.UnsupportedOperation: fileno
        patcher = mock.patch('sys.stdout.fileno')
        self.fileno_mock = patcher.start()
        self.fileno_mock.return_value = 1
        self.addCleanup(patcher.stop)

        patcher = mock.patch('os.isatty')
        self.isatty_mock = patcher.start()
        self.isatty_mock.return_value = False
        self.addCleanup(patcher.stop)

        # Create a snap
        main(['init'])
        main(['snap'])
        snap_file = glob.glob('*.snap')[0]

        # Upload
        with mock.patch('snapcraft.storeapi.StatusTracker') as mock_tracker:
            main(['upload', snap_file])

        self.assertIn(
            'Uploading my-snap_0_amd64.snap.\n'
            'Revision 1 of \'my-snap\' created.',
            self.fake_logger.output)

        mock_upload.assert_called_once_with('my-snap', snap_file)

    def test_upload_without_login_must_raise_exception(self):
        snap_path = os.path.join(
            os.path.dirname(tests.__file__), 'data',
            'test-snap.snap')
        with self.assertRaises(SystemExit):
            main(['upload', snap_path])
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    def test_upload_nonexisting_snap_must_raise_exception(self):
        with self.assertRaises(SystemExit):
            main(['upload', 'test-unexisting-snap'])
