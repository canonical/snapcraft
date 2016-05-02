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

import contextlib
import os
import unittest

import requests
import testscenarios

from snapcraft import (
    _store,
    storeapi,
)
from snapcraft.storeapi import _upload
from snapcraft.tests import store_tests


load_tests = testscenarios.load_tests_apply_scenarios


class TestUploadNoLogin(store_tests.TestCase):

    def test_upload_without_credentials(self):
        snap_path, snap_name = self.create_snap('notevenregistered')

        self.assertRaises(storeapi.InvalidCredentialsError,
                          self.upload, snap_path, snap_name)


class UploadTestCase(store_tests.RecordedTestCase):

    def setUp(self):
        super().setUp()
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

        class FakeUpdown(self.preserved_session_class):

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


class TestExceptions(unittest.TestCase):

    def test_mismatcherror(self):
        self.assertEqual('SHA512 checksum for path is not sha.',
                         str(storeapi.SHAMismatchError('path', 'sha')))


class FakeSession(object):

    def __init__(self, exc):
        self.exc = exc

    def get(self, url):
        raise self.exc()


class ScanStatusTestCase(unittest.TestCase):

    def test_is_scan_complete_for_none(self):
        self.assertFalse(_upload.is_scan_completed(None))

    def test_is_scan_complete_for_response_not_ok(self):
        class Response(object):
            ok = False
        self.assertFalse(_upload.is_scan_completed(Response()))

    def get_scan_status(self, exc):
        raiser = FakeSession(exc)
        return _upload.get_scan_status(raiser, 'foo')

    def test_get_status_connection_error(self):
        self.assertIsNone(self.get_scan_status(requests.ConnectionError))

    def test_get_status_http_error(self):
        self.assertIsNone(self.get_scan_status(requests.HTTPError))


class TestUpload_store(store_tests.TestCase):

    def test_upload_unknwon_file(self):
        self.login()
        self.addCleanup(self.logout)
        exc = self.assertRaises(
            FileNotFoundError,
            _store.upload, 'I-dont-exist')
        self.assertEqual('I-dont-exist', str(exc))

    def test_upload_fails(self):
        self.login()
        self.addCleanup(self.logout)
        path, name = self.create_snap('basic')

        def you_failed(*args):
            return dict(success=False, errors=['You failed'])
        self.addCleanup(
            setattr, storeapi.SCAClient, 'upload', storeapi.SCAClient.upload)
        storeapi.SCAClient.upload = you_failed
        _store.upload(path)
        self.assertIn('Upload did not complete', self.logger.output)
        self.assertIn('Some errors were detected:\n\nYou failed\n',
                      self.logger.output)

    def test__upload_files_fails_review(self):
        # Pretend the binary upload succeeded
        def uploaded(name, data):
            class Response(object):
                ok = True

                def json(*args):
                    return dict(status_url='foobar')
            return Response()
        self.store.upload_snap = uploaded

        # Pretend the snap upload failed the scan
        class FailingExecutor(object):

            def done(self):
                return True

            def result(self):
                return True, dict(message='You failed review')

        class Spawner(object):

            def submit(*args):
                return FailingExecutor()

        @contextlib.contextmanager
        def executor(*args, **kwargs):
            yield Spawner()

        self.addCleanup(
            setattr, _upload, 'ThreadPoolExecutor', _upload.ThreadPoolExecutor)
        _upload.ThreadPoolExecutor = executor
        result = {}
        res = _upload._upload_files(self.store, 'foo', {}, result)
        self.assertEqual(['You failed review'], res['errors'])

    def test__upload_files_review_too_long(self):
        # Pretend the binary upload succeeded
        def uploaded(name, data):
            class Response(object):
                ok = True

                def json(*args):
                    return dict(status_url='foobar',
                                web_status_url='See there')
            return Response()
        self.store.upload_snap = uploaded

        # Pretend the snap upload failed the scan
        class FailingExecutor(object):

            def done(self):
                return True

            def result(self):
                return False, dict(message='You failed review')

        class Spawner(object):

            def submit(*args):
                return FailingExecutor()

        @contextlib.contextmanager
        def executor(*args, **kwargs):
            yield Spawner()

        self.addCleanup(
            setattr, _upload, 'ThreadPoolExecutor', _upload.ThreadPoolExecutor)
        _upload.ThreadPoolExecutor = executor
        result = {}
        res = _upload._upload_files(self.store, 'foo', {}, result)
        self.assertEqual(['Package scan took too long.',
                          'Please check the status later at: See there.'],
                         res['errors'])

    def test__upload_files_other_error(self):
        # Pretend the binary failed to upload
        def uploaded(name, data):
            class Response(object):
                ok = False
                reason = 'Some'
                text = 'Full Moon'
            return Response()
        self.store.upload_snap = uploaded
        result = {}
        res = _upload._upload_files(self.store, 'foo', {}, result)
        self.assertEqual(['Full Moon'],
                         res['errors'])
        self.assertIn('There was an error uploading the application.\n'
                      'Reason: Some\n'
                      'Text: Full Moon',
                      self.logger.output)
