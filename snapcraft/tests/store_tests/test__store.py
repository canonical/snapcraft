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
import getpass
import os
import uuid

import testscenarios

from snapcraft import (
    _store,
    storeapi,
)
from snapcraft.storeapi import _upload
from snapcraft.tests import store_tests


load_tests = testscenarios.load_tests_apply_scenarios


class TestStore(store_tests.RecordedTestCase):
    """Test the store api from _store.

    This covers the store API used by snapcraft.
    """
    def login(self, email=None, password=None):
        email = email or os.getenv('TEST_USER_EMAIL',
                                   'u1test+snapcraft@canonical.com')
        env_password = os.getenv('TEST_USER_PASSWORD', None)
        password = password or env_password

        canned_inputs = [email, '']

        def fake_input(prompt):
            return canned_inputs.pop(0)
        orig_input = input

        def fake_getpass(prompt):
            return password
        orig_getpass = getpass.getpass
        try:
            setattr(_store, 'input', fake_input)
            setattr(getpass, 'getpass', fake_getpass)
            # FIXME: Find a way to test one-time-passwords (otp)
            # -- vila 2016-04-11
            return _store.login()
        finally:
            setattr(_store, 'input', orig_input)
            setattr(getpass, 'getpass', orig_getpass)

    def logout(self):
        return _store.logout()


class TestLoginLogout(TestStore):

    def test_successful_login(self):
        self.addCleanup(self.logout)
        self.assertTrue(self.login())

    def test_failed_login(self):
        self.assertFalse(self.login(password='wrongpassword'))


class TestRegister(TestStore):

    # FIXME: The store doesn't provide a way to unregister a name *and*
    # registrations are rate-limited for a given user. We work around that by
    # creating unique names and assuming we only run against staging or local
    # dev instances -- vila 2016-04-08
    def test_successful_register(self):
        self.login()
        self.addCleanup(self.logout)
        uniq_name = 'delete-me-{}'.format(str(uuid.uuid4().int)[:32])
        self.assertTrue(_store.register_name(uniq_name))

    def test_register_without_login(self):
        self.assertFalse(_store.register_name('foobar'))

    def test_register_registered_name(self):
        self.login()
        self.addCleanup(self.logout)
        self.assertFalse(_store.register_name('femto'))


class TestUpload(TestStore):

    def test_upload_works(self):
        self.login()
        self.addCleanup(self.logout)
        snap_path, snap_name = self.create_snap('basic')
        self.assertTrue(_store.upload(snap_path))


class TestUploadWithFakes(store_tests.TestCase):
    """Tests with fakes to complete coverage."""

    def test_upload_without_login(self):
        path, name = self.create_snap('basic')
        _store.upload(path)
        self.assertIn('No valid credentials found', self.logger.output)

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


class TestDownload(TestStore):

    def test_download_without_login(self):
        pkg_name = 'femto'
        _store.download(pkg_name, 'stable', 'downloaded.snap', 'amd64')
        self.assertIn('No valid credentials found', self.logger.output)

    def test_download_not_found(self):
        self.login()
        self.addCleanup(self.logout)
        exc = self.assertRaises(
            RuntimeError,
            _store.download, 'gloo', 'bee', 'downloaded.snap', 'amd64')
        self.assertEqual(
            'Snap gloo for amd64 cannot be found in the bee channel',
            str(exc))

    def test_download_mismatch(self):
        self.login()
        self.addCleanup(self.logout)
        self.addCleanup(setattr, storeapi.SCAClient, 'download',
                        storeapi.SCAClient.download)

        def raise_not_a_sha(*args):
            raise storeapi.SHAMismatchError('downloaded.snap', 'not-a-sha')
        storeapi.SCAClient.download = raise_not_a_sha
        exc = self.assertRaises(
            RuntimeError,
            _store.download, 'femto', 'stable', 'downloaded.snap', 'amd64')
        self.assertEqual(
            'Failed to download femto at downloaded.snap (mismatched SHA)',
            str(exc))
