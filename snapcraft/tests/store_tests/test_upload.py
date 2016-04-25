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
import unittest

import fixtures
import requests
import testscenarios

from snapcraft import storeapi
from snapcraft.storeapi import _upload
from snapcraft.tests import store_tests


load_tests = testscenarios.load_tests_apply_scenarios


class TestUploadNoLogin(store_tests.TestCase):

    scenarios = (('OAuth', dict(with_macaroons=False)),
                 ('macaroons', dict(with_macaroons=True)),
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
        snap_path, snap_name = self.create_snap('unregistered')

        self.assertRaises(storeapi.InvalidCredentials,
                          self.upload, snap_path, snap_name)


class UploadTestCase(store_tests.TestCase):

    scenarios = (('OAuth', dict(with_macaroons=False)),
                 ('macaroons', dict(with_macaroons=True)),
                 )

    def setUp(self):
        super().setUp()
        if self.with_macaroons:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        else:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))
        # Some tests will override the following internals
        self.preserved_upload_files = _upload.upload_files
        self.addCleanup(
            setattr, _upload, 'upload_files', self.preserved_upload_files)

        self.preserved__upload_files = _upload._upload_files
        self.addCleanup(
            setattr, _upload, '_upload_files', self.preserved__upload_files)
        self.addCleanup(self.logout)
        self.login()

    def test_upload_works(self):
        snap_path, snap_name = self.create_snap('basic')

        resp = self.upload(snap_path, snap_name)
        # FIXME: Each test user need his own registered snap names for the test
        # to succeed -- vila 2016-04-12
        test_user = os.environ.get('TEST_USER_EMAIL',
                                   'u1test+snapcraft@canonical.com')
        registered = (test_user == 'u1test+snapcraft@canonical.com')
        if not registered:
            self.expectFailure('basic is registered by someone else',
                               self.assertTrue, resp['success'])
        self.assertTrue(
            resp['success'],
            'upload failed with: {}'.format(resp.get('errors', '')))
        # The exact content vary but the keys should exist when the upload
        # succeeds
        self.assertTrue('application_url' in resp)
        self.assertTrue('revision' in resp)

    def test_upload_app_failed(self):
        # Make upload_app catch an error raised by _upload_files
        def raise_error(*args, **kwargs):
            raise Exception('that error')
        _upload._upload_files = raise_error

        snap_path, snap_name = self.create_snap('basic')

        resp = self.upload(snap_path, snap_name)
        self.assertFalse(resp['success'])
        self.assertEqual(['that error'], resp['errors'])
        log = self.logger.output
        self.assertIn('There was an error uploading the application', log)

    def inject_response(self, response):
        """Inject a predefined response when upload_files calls updown."""

        class FakeUpdown(requests.Session):

            def post(self, url, data=None, json=None, **kwargs):
                if callable(response):
                    return response()
                else:
                    return response

        def failing_upload_files(binary_filename, session):
            return self.preserved_upload_files(binary_filename, FakeUpdown())

        return failing_upload_files

    def test_upload_files_failed_unexpectedly(self):
        def raise_error(*args, **kwargs):
            raise Exception('this error')

        _upload.upload_files = self.inject_response(raise_error)

        snap_path, snap_name = self.create_snap('basic')

        resp = self.upload(snap_path, snap_name)
        self.assertFalse(resp['success'])
        self.assertEqual(['this error'], resp['errors'])
        self.assertIn('An unexpected error was found while uploading files.',
                      self.logger.output)

    def test_upload_files_failed_from_server(self):
        failed = requests.Response()
        failed.status_code = 500
        failed.reason = 'Bad Reason'

        _upload.upload_files = self.inject_response(failed)

        snap_path, snap_name = self.create_snap('basic')

        resp = self.upload(snap_path, snap_name)
        self.assertFalse(resp['success'])
        self.assertEqual([''], resp['errors'])
        self.assertIn('There was an error uploading the package.',
                      self.logger.output)
        self.assertIn('Bad Reason',
                      self.logger.output)


class FakeSession(object):

    def __init__(self, exc):
        self.exc = exc

    def get(self, url):
        raise self.exc()


class ScanStatusTestCase(unittest.TestCase):

    def test_is_scan_complete_for_none(self):
        self.assertFalse(_upload.is_scan_completed(None))

    def get_scan_status(self, exc):
        raiser = FakeSession(exc)
        return _upload.get_scan_status(raiser, 'foo')

    def test_get_status_connection_error(self):
        self.assertIsNone(self.get_scan_status(requests.ConnectionError))

    def test_get_status_http_error(self):
        self.assertIsNone(self.get_scan_status(requests.HTTPError))
