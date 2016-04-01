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
from snapcraft import tests
from snapcraft.main import main


class UploadCommandTestCase(tests.TestCase):

    def _patch_snap_yaml(self, snap_name):
        meta_path = os.path.join(os.getcwd(), 'squashfs-root', 'meta')
        os.makedirs(meta_path)
        with open(os.path.join(meta_path, 'snap.yaml'), 'w') as yaml_file:
            yaml_file.write('name: {}\n'.format(snap_name))

        temp_dir = snapcraft._store.tempfile.TemporaryDirectory

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

    def test_upload_nonexisting_snap_must_raise_exception(self):
        with self.assertRaises(SystemExit) as raised:
            main(['upload', 'unexisting.snap'])

        self.assertEqual('unexisting.snap', str(raised.exception))

    def test_upload_existing_snap(self):
        patcher = mock.patch('snapcraft.storeapi.upload')
        mock_upload = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.config.load_config')
        mock_load_config = patcher.start()
        self.addCleanup(patcher.stop)
        mock_load_config.return_value = 'test config'

        patcher = mock.patch('subprocess.check_call')
        mock_check_call = patcher.start()
        self.addCleanup(patcher.stop)

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        open('test.snap', 'w').close()

        self._patch_snap_yaml('snaptestname')
        main(['upload', 'test.snap'])

        mock_check_call.assert_called_once_with(
            ['unsquashfs', '-d', os.path.join(os.getcwd(), 'squashfs-root'),
             'test.snap', '-e', os.path.join('meta', 'snap.yaml')])
        self.assertEqual(
            'Uploading existing test.snap.\n', fake_logger.output)

        mock_upload.assert_called_once_with(
            'test.snap',
            'snaptestname',
            config='test config')
