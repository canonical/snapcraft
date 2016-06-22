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

    def _patch_snap_yaml(self, snap_name):
        meta_path = os.path.join(os.getcwd(), 'squashfs-root', 'meta')
        os.makedirs(meta_path)
        with open(os.path.join(meta_path, 'snap.yaml'), 'w') as yaml_file:
            yaml_file.write('name: {}\n'.format(snap_name))

        temp_dir = snapcraft.storeapi.tempfile.TemporaryDirectory

        def create_patch(method, return_value):
            patcher = mock.patch.object(temp_dir, method)
            mock_method = patcher.start()
            mock_method.return_value = return_value
            self.addCleanup(patcher.stop)

        create_patch('_cleanup', None)
        create_patch('__enter__', os.getcwd())
        create_patch('__exit__', False)

    def test_upload_without_snap_must_raise_exception(self):
        with self.assertRaises(docopt.DocoptExit) as raised:
            main(['upload'])

        self.assertTrue('Usage:' in str(raised.exception))

    def test_upload_snap(self):
        patcher = mock.patch.object(storeapi.StoreClient, 'upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)
        mock_upload.return_value = {
            'success': True,
            'revision': 'test-revision',
            'application_url': 'test-url'
        }

        open('test.snap', 'w').close()

        self._patch_snap_yaml('snaptestname')
        main(['upload', 'test.snap'])

        self.assertEqual(
            'Uploading test.snap.\n'
            'Application uploaded successfully (as revision test-revision)\n'
            'Please check out the application at: test-url\n\n',
            self.fake_logger.output)

        mock_upload.assert_called_once_with('test.snap')

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
