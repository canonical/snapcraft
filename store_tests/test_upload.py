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

import fixtures
import testscenarios

from snapcraft.storeapi import _upload
import store_tests


load_tests = testscenarios.load_tests_apply_scenarios


class UploadTestCase(store_tests.TestCase):

    scenarios = (('OAuth', dict(with_macaroons=False)),
                 # ('macaroons', dict(with_macaroons=True)),
                 )

    def setUp(self):
        super().setUp()
        if self.with_macaroons:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        else:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))

    def test_upload_without_login(self):
        snap_path, snap_name = self.create_snap('unregsitered')

        resp = self.upload(snap_path, snap_name)
        self.assertFalse(resp['success'])

        log = self.logger.output
        self.assertIn('Upload failed', log)
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?', log)

    def test_upload_with_login(self):
        self.addCleanup(self.logout)
        self.login()

        snap_path, snap_name = self.create_snap('basic')

        resp = self.upload(snap_path, snap_name)
        # FIXME: Each test user need his own registered snap names for the test
        # to succeed -- vila 2016-04-12
        registered = (
            os.environ.get('TEST_USER_EMAIL')
            == 'u1test+snapcraft@canonical.com')
        if not registered:
            self.expectFailure('basic is registered by someone else',
                               self.assertTrue, resp['success'])
        self.assertTrue(resp['success'])
        # The exact content vary but the keys should exist when the upload
        # succeeds
        self.assertTrue('application_url' in resp)
        self.assertTrue('revision' in resp)

    def test_upload_app_failed(self):
        # Make upload_app catch an error raised by _upload_files
        def raise_error(*args, **kwargs):
            raise Exception('that error')
        self.addCleanup(
            setattr, _upload, '_upload_files', _upload._upload_files)
        _upload._upload_files = raise_error

        self.addCleanup(self.logout)
        self.login()
        snap_path, snap_name = self.create_snap('basic')

        resp = self.upload(snap_path, snap_name)
        self.assertFalse(resp['success'])
        self.assertEqual(['that error'], resp['errors'])
        log = self.logger.output
        self.assertIn('There was an error uploading the application', log)
