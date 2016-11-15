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

import json
import logging
import os
from textwrap import dedent
from unittest import mock

import fixtures
import pymacaroons

from snapcraft import (
    config,
    storeapi,
    tests
)
from snapcraft.storeapi import errors
from snapcraft.tests import fixture_setup


class LoginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_login_successful(self):
        self.client.login(
            'dummy email',
            'test correct password')
        conf = config.Config()
        self.assertIsNotNone(conf.get('macaroon'))
        self.assertIsNotNone(conf.get('unbound_discharge'))

    def test_login_successful_with_one_time_password(self):
        self.client.login(
            'dummy email',
            'test correct password',
            'test correct one-time password')
        conf = config.Config()
        self.assertIsNotNone(conf.get('macaroon'))
        self.assertIsNotNone(conf.get('unbound_discharge'))

    def test_failed_login_with_wrong_password(self):
        with self.assertRaises(errors.StoreAuthenticationError):
            self.client.login('dummy email', 'wrong password')

        self.assertTrue(config.Config().is_empty())

    def test_failed_login_requires_one_time_password(self):
        with self.assertRaises(errors.StoreTwoFactorAuthenticationRequired):
            self.client.login('dummy email', 'test requires 2fa')

        self.assertTrue(config.Config().is_empty())

    def test_failed_login_with_wrong_one_time_password(self):
        with self.assertRaises(errors.StoreAuthenticationError):
            self.client.login(
                'dummy email',
                'test correct password',
                'wrong one-time password')

        self.assertTrue(config.Config().is_empty())

    def test_failed_login_with_invalid_json(self):
        with self.assertRaises(errors.StoreAuthenticationError):
            self.client.login('dummy email', 'test 401 invalid json')

        self.assertTrue(config.Config().is_empty())


class DownloadTestCase(tests.TestCase):

    # sha512 of snapcraft/tests/data/test-snap.snap
    EXPECTED_SHA512 = (
        '69D57DCACF4F126592D4E6FF689AD8BB8A083C7B9FE44F6E738EF'
        'd22a956457f14146f7f067b47bd976cf0292f2993ad864ccb498b'
        'fda4128234e4c201f28fe9')

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_download_unexisting_snap_raises_exception(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.SnapNotFoundError) as e:
            self.client.download(
                'unexisting-snap', 'test-channel', 'dummy', 'test-arch')
        self.assertEqual(
            "Snap 'unexisting-snap' for 'test-arch' cannot be found in "
            "the 'test-channel' channel.",
            str(e.exception))

    def test_download_snap(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login('dummy', 'test correct password')
        download_path = os.path.join(self.path, 'test-snap.snap')
        self.client.download(
            'test-snap', 'test-channel', download_path)
        self.assertIn(
            'Successfully downloaded test-snap at {}'.format(download_path),
            self.fake_logger.output)

    def test_download_already_downloaded_snap(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login('dummy', 'test correct password')
        download_path = os.path.join(self.path, 'test-snap.snap')
        # download first time.
        self.client.download(
            'test-snap', 'test-channel', download_path)
        # download again.
        self.client.download(
            'test-snap', 'test-channel', download_path)
        self.assertIn(
            'Already downloaded test-snap at {}'.format(download_path),
            self.fake_logger.output)

    def test_download_on_sha_mismatch(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login('dummy', 'test correct password')
        download_path = os.path.join(self.path, 'test-snap.snap')
        # Write a wrong file in the download path.
        open(download_path, 'w').close()
        self.client.download(
            'test-snap', 'test-channel', download_path)
        self.assertIn(
            'Successfully downloaded test-snap at {}'.format(download_path),
            self.fake_logger.output)

    def test_download_with_hash_mismatch_raises_exception(self):
        self.client.login('dummy', 'test correct password')
        download_path = os.path.join(self.path, 'test-snap.snap')
        with self.assertRaises(errors.SHAMismatchError):
            self.client.download(
                'test-snap-with-wrong-sha', 'test-channel', download_path)


class PushSnapBuildTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_push_snap_build_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.push_snap_build('snap-id', 'dummy')

    def test_push_snap_build_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.client.push_snap_build('snap-id', 'dummy')
        self.assertFalse(self.fake_store.needs_refresh)

    def test_push_snap_build_not_implemented(self):
        # If the "enable_snap_build" feature switch is off in the store, we
        # will get a descriptive error message.
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreSnapBuildError) as raised:
            self.client.push_snap_build('snap-id', 'test-not-implemented')
        self.assertEqual(
            str(raised.exception),
            'Could not assert build: The snap-build assertions are '
            'currently disabled.')

    def test_push_snap_build_invalid_data(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreSnapBuildError) as raised:
            self.client.push_snap_build('snap-id', 'test-invalid-data')
        self.assertEqual(
            str(raised.exception),
            'Could not assert build: The snap-build assertion is not valid.')

    def test_push_snap_build_unexpected_data(self):
        # The endpoint in SCA would never return plain/text, however anything
        # might happen in the internet, so we are a little defensive.
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreSnapBuildError) as raised:
            self.client.push_snap_build('snap-id', 'test-unexpected-data')
        self.assertEqual(
            str(raised.exception),
            'Could not assert build: 500 Internal Server Error')

    def test_push_snap_build_successfully(self):
        self.client.login('dummy', 'test correct password')
        # No exception will be raised if this is successful.
        self.client.push_snap_build('snap-id', 'dummy')


class GetAccountInformationTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_get_account_information_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.get_account_information()

    def test_get_account_information_successfully(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual({
            'account_id': 'abcd',
            'account_keys': [],
            'snaps': {'16': {
                'basic': {'snap-id': 'snap-id'},
                'ubuntu-core': {'snap-id': 'good'}}}},
            self.client.get_account_information())

    def test_get_account_information_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.assertEqual({
            'account_id': 'abcd',
            'account_keys': [],
            'snaps': {'16': {
                'basic': {'snap-id': 'snap-id'},
                'ubuntu-core': {'snap-id': 'good'}}}},
            self.client.get_account_information())
        self.assertFalse(self.fake_store.needs_refresh)


class RegisterKeyTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_register_key_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.register_key('dummy')

    def test_register_key_successfully(self):
        self.client.login('dummy', 'test correct password')
        # No exception will be raised if this is successful.
        self.client.register_key(dedent('''\
            name: default
            public-key-sha3-384: abcd
            '''))

    def test_register_key_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.client.register_key(dedent('''\
            name: default
            public-key-sha3-384: abcd
            '''))
        self.assertFalse(self.fake_store.needs_refresh)

    def test_not_implemented(self):
        # If the enable_account_key feature switch is off in the store, we
        # will get a 501 Not Implemented response.
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreKeyRegistrationError) as raised:
            self.client.register_key('test-not-implemented')
        self.assertEqual(
            str(raised.exception),
            'Key registration failed: 501 Not Implemented')

    def test_invalid_data(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreKeyRegistrationError) as raised:
            self.client.register_key('test-invalid-data')
        self.assertEqual(
            str(raised.exception),
            'Key registration failed: '
            'The account-key-request assertion is not valid.')


class RegisterTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_register_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.register('dummy')

    def test_register_name_successfully(self):
        self.client.login('dummy', 'test correct password')
        # No exception will be raised if this is succesful
        self.client.register('test-good-snap-name')

    def test_register_private_name_successfully(self):
        self.client.login('dummy', 'test correct password')
        # No exception will be raised if this is succesful
        self.client.register('test-good-snap-name', is_private=True)

    def test_register_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.client.register('test-good-snap-name')
        self.assertFalse(self.fake_store.needs_refresh)

    def test_already_registered(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreRegistrationError) as raised:
            self.client.register('test-already-registered-snap-name')
        self.assertEqual(
            str(raised.exception),
            "The name 'test-already-registered-snap-name' is already taken."
            "\n\n"
            "We can if needed rename snaps to ensure they match the "
            "expectations of most users. If you are the publisher most users "
            "expect for 'test-already-registered-snap-name' then claim the "
            "name at 'https://myapps.com/register-name/'")

    def test_register_a_reserved_name(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreRegistrationError) as raised:
            self.client.register('test-reserved-snap-name')
        self.assertEqual(
            str(raised.exception),
            "The name 'test-reserved-snap-name' is reserved."
            "\n\n"
            "If you are the publisher most users expect for "
            "'test-reserved-snap-name' then please claim the "
            "name at 'https://myapps.com/register-name/'")

    def test_registering_too_fast_in_a_row(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreRegistrationError) as raised:
            self.client.register('test-too-fast')
        self.assertEqual(
            str(raised.exception),
            'You must wait 177 seconds before trying to register your '
            'next snap.')

    def test_unhandled_registration_error_path(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreRegistrationError) as raised:
            self.client.register('snap-name-no-clear-error')
        self.assertEqual(str(raised.exception), 'Registration failed.')


class ValidationsTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_get_success(self):
        self.client.login('dummy', 'test correct password')
        expected = [{
            "approved-snap-id": "snap-id-1",
            "approved-snap-revision": "3",
            "approved-snap-name": "snap-1",
            "authority-id": "dev-1",
            "series": "16",
            "sign-key-sha3-384": "1234567890",
            "snap-id": "snap-id-gating",
            "timestamp": "2016-09-19T21:07:27.756001Z",
            "type": "validation",
            "revoked": "false",
            "required": True,
        }, {
            "approved-snap-id": "snap-id-2",
            "approved-snap-revision": "5",
            "approved-snap-name": "snap-2",
            "authority-id": "dev-1",
            "series": "16",
            "sign-key-sha3-384": "1234567890",
            "snap-id": "snap-id-gating",
            "timestamp": "2016-09-19T21:07:27.756001Z",
            "type": "validation",
            "revoked": "false",
            "required": False,
        }, {
            "approved-snap-id": "snap-id-3",
            "approved-snap-revision": "-",
            "approved-snap-name": "snap-3",
            "authority-id": "dev-1",
            "series": "16",
            "sign-key-sha3-384": "1234567890",
            "snap-id": "snap-id-gating",
            "timestamp": "2016-09-19T21:07:27.756001Z",
            "type": "validation",
            "revoked": "false",
            "required": True,
        }]
        result = self.client.get_validations('good')
        self.assertEqual(result, expected)

    def test_get_bad_response(self):
        self.client.login('dummy', 'test correct password')

        with self.assertRaises(errors.StoreValidationError) as err:
            self.client.get_validations('bad')

        expected = ("Received error 200: 'Invalid response from the server'")
        self.assertEqual(str(err.exception), expected)
        self.assertIn(
            'Invalid response from the server', self.fake_logger.output)

    def test_get_error_response(self):
        self.client.login('dummy', 'test correct password')
        expected = []

        with self.assertRaises(errors.StoreValidationError) as err:
            self.client.get_validations('err')

        expected = ("Received error 503: 'error'")
        self.assertEqual(str(err.exception), expected)

    def test_push_success(self):
        self.client.login('dummy', 'test correct password')
        assertion = json.dumps({'foo': 'bar'}).encode('utf-8')

        result = self.client.push_validation('good', assertion)

        expected = {'assertion': '{"foo": "bar"}'}
        self.assertEqual(result, expected)

    def test_push_bad_response(self):
        self.client.login('dummy', 'test correct password')
        assertion = json.dumps({'foo': 'bar'}).encode('utf-8')

        with self.assertRaises(errors.StoreValidationError) as err:
            self.client.push_validation('bad', assertion)

        expected = ("Received error 200: 'Invalid response from the server'")
        self.assertEqual(str(err.exception), expected)
        self.assertIn(
            'Invalid response from the server', self.fake_logger.output)

    def test_push_error_response(self):
        self.client.login('dummy', 'test correct password')
        assertion = json.dumps({'foo': 'bar'}).encode('utf-8')

        with self.assertRaises(errors.StoreValidationError) as err:
            self.client.push_validation('err', assertion)

        expected = ("Received error 501: 'error'")
        self.assertEqual(str(err.exception), expected)


class UploadTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()
        self.snap_path = os.path.join(
            os.path.dirname(tests.__file__), 'data',
            'test-snap.snap')
        # These should eventually converge to the same module
        pbars = (
            'snapcraft.storeapi._upload.ProgressBar',
            'snapcraft.storeapi.ProgressBar',
        )
        for pbar in pbars:
            patcher = mock.patch(pbar, new=tests.SilentProgressBar)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_upload_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.upload('test-snap', self.snap_path)

    def test_upload_snap(self):
        self.client.login('dummy', 'test correct password')
        tracker = self.client.upload('test-snap', self.snap_path)
        self.assertTrue(isinstance(tracker, storeapi.StatusTracker))
        result = tracker.track()
        expected_result = {
            'code': 'ready_to_release',
            'revision': '1',
            'url': '/dev/click-apps/5349/rev/1',
            'can_release': True,
            'processed': True
        }
        self.assertEqual(result, expected_result)

        # This should not raise
        tracker.raise_for_code()

    def test_upload_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        tracker = self.client.upload('test-snap', self.snap_path)
        result = tracker.track()
        expected_result = {
            'code': 'ready_to_release',
            'revision': '1',
            'url': '/dev/click-apps/5349/rev/1',
            'can_release': True,
            'processed': True
        }
        self.assertEqual(result, expected_result)

        # This should not raise
        tracker.raise_for_code()

        self.assertFalse(self.fake_store.needs_refresh)

    def test_upload_snap_fails_due_to_upload_fail(self):
        # Tells the fake updown server to return a 5xx response
        self.useFixture(fixtures.EnvironmentVariable('UPDOWN_BROKEN', '1'))

        self.client.login('dummy', 'test correct password')

        with self.assertRaises(errors.StoreUploadError) as raised:
            self.client.upload('test-snap', self.snap_path)

        self.assertEqual(
            str(raised.exception),
            'There was an error uploading the package.\n'
            'Reason: \'Internal Server Error\'\n'
            'Text: \'Broken\'')

    def test_upload_snap_requires_review(self):
        self.client.login('dummy', 'test correct password')
        tracker = self.client.upload('test-review-snap', self.snap_path)
        self.assertTrue(isinstance(tracker, storeapi.StatusTracker))
        result = tracker.track()
        expected_result = {
            'code': 'need_manual_review',
            'revision': '1',
            'url': '/dev/click-apps/5349/rev/1',
            'can_release': False,
            'processed': True
        }
        self.assertEqual(result, expected_result)

        with self.assertRaises(errors.StoreReviewError):
            tracker.raise_for_code()

    def test_upload_unregistered_snap(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StorePushError) as raised:
            self.client.upload('test-snap-unregistered', self.snap_path)
        self.assertEqual(
            str(raised.exception),
            'Sorry, try `snapcraft register '
            'test-snap-unregistered` before pushing again.')

    def test_upload_with_invalid_credentials_raises_exception(self):
        conf = config.Config()
        conf.set('macaroon', 'inval"id')
        conf.save()
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.upload('test-snap', self.snap_path)


class ReleaseTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_release_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.release('test-snap', '19', ['beta'])

    def test_release_snap(self):
        self.client.login('dummy', 'test correct password')
        channel_map = self.client.release('test-snap', '19', ['beta'])
        expected_channel_map = {
            'opened_channels': ['beta'],
            'channel_map': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'revision': 19, 'channel': 'beta', 'version': '0',
                 'info': 'specific'},
                {'channel': 'edge', 'info': 'tracking'}
            ]
        }
        self.assertEqual(channel_map, expected_channel_map)

    def test_release_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        channel_map = self.client.release('test-snap', '19', ['beta'])
        expected_channel_map = {
            'opened_channels': ['beta'],
            'channel_map': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'revision': 19, 'channel': 'beta', 'version': '0',
                 'info': 'specific'},
                {'channel': 'edge', 'info': 'tracking'}
            ]
        }
        self.assertEqual(channel_map, expected_channel_map)
        self.assertFalse(self.fake_store.needs_refresh)

    def test_release_snap_to_invalid_channel(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreReleaseError) as raised:
            self.client.release('test-snap', '19', ['alpha'])

        self.assertEqual(
            str(raised.exception),
            'Not a valid channel: alpha')

    def test_release_unregistered_snap(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreReleaseError) as raised:
            self.client.release('test-snap-unregistered', '19', ['alpha'])

        self.assertEqual(
            str(raised.exception),
            'Sorry, try `snapcraft register test-snap-unregistered` '
            'before trying to release or choose an existing '
            'revision.')

    def test_release_with_invalid_credentials_raises_exception(self):
        conf = config.Config()
        conf.set('macaroon', 'inval"id')
        conf.save()
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.release('test-snap', '10', ['beta'])


class CloseChannelsTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_close_requires_login(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.close_channels('snap-id', ['dummy'])

    def test_close_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.client.close_channels('snap-id', ['dummy'])
        self.assertFalse(self.fake_store.needs_refresh)

    def test_close_invalid_data(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreChannelClosingError) as raised:
            self.client.close_channels('snap-id', ['invalid'])
        self.assertEqual(
            str(raised.exception),
            "Could not close channel: The 'channels' field content "
            "is not valid.")

    def test_close_unexpected_data(self):
        # The endpoint in SCA would never return plain/text, however anything
        # might happen in the internet, so we are a little defensive.
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreChannelClosingError) as raised:
            self.client.close_channels('snap-id', ['unexpected'])
        self.assertEqual(
            str(raised.exception),
            'Could not close channel: 500 Internal Server Error')

    def test_close_broken_store_plain(self):
        # If the contract is broken by the Store, users will be have additional
        # debug information available.
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreChannelClosingError) as raised:
            self.client.close_channels('snap-id', ['broken-plain'])
        self.assertEqual(
            str(raised.exception),
            'Could not close channel: 200 OK')
        self.assertEqual([
            'Invalid response from the server on channel closing:',
            '200 OK',
            'b\'plain data\'',
            ], self.fake_logger.output.splitlines()[-3:])

    def test_close_broken_store_json(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreChannelClosingError) as raised:
            self.client.close_channels('snap-id', ['broken-json'])
        self.assertEqual(
            str(raised.exception),
            'Could not close channel: 200 OK')
        self.assertEqual([
            'Invalid response from the server on channel closing:',
            '200 OK',
            'b\'{"closed_channels": ["broken-json"]}\'',
            ], self.fake_logger.output.splitlines()[-3:])

    def test_close_successfully(self):
        # Successfully closing a channels returns 'closed_channels'
        # and 'channel_maps' from the Store.
        self.client.login('dummy', 'test correct password')
        closed_channels, channel_maps = self.client.close_channels(
            'snap-id', ['beta'])
        self.assertEqual(['beta'], closed_channels)
        self.assertEqual({
            'amd64': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'channel': 'beta', 'info': 'specific',
                 'revision': 42, 'version': '1.1'},
                {'channel': 'edge', 'info': 'tracking'}]
        }, channel_maps)


class MacaroonsTestCase(tests.TestCase):

    def test_invalid_macaroon_root_raises_exception(self):
        conf = config.Config()
        conf.set('macaroon', 'inval"id')
        conf.save()
        with self.assertRaises(errors.InvalidCredentialsError):
            storeapi._macaroon_auth(conf)

    def test_invalid_discharge_raises_exception(self):
        conf = config.Config()
        conf.set('macaroon', pymacaroons.Macaroon().serialize())
        conf.set('unbound_discharge', 'inval*id')
        conf.save()
        with self.assertRaises(errors.InvalidCredentialsError):
            storeapi._macaroon_auth(conf)


class GetSnapHistoryTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()
        self.expected = [{
            'series': ['16'],
            'channels': [],
            'version': '2.0.1',
            'timestamp': '2016-09-27T19:23:40Z',
            'current_channels': ['beta', 'edge'],
            'arch': 'i386',
            'revision': 2
        }, {
            'series': ['16'],
            'channels': ['stable', 'edge'],
            'version': '2.0.2',
            'timestamp': '2016-09-27T18:38:43Z',
            'current_channels': ['stable', 'candidate', 'beta'],
            'arch': 'amd64',
            'revision': 1,
        }]

    def test_get_snap_history_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.get_snap_history('basic')

    def test_get_snap_history_successfully(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(self.expected, self.client.get_snap_history('basic'))

    def test_get_snap_history_filter_by_series(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(
            self.expected,
            self.client.get_snap_history('basic', series='16'))

    def test_get_snap_history_filter_by_arch(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(
            [rev for rev in self.expected if rev['arch'] == 'amd64'],
            self.client.get_snap_history('basic', arch='amd64'))

    def test_get_snap_history_filter_by_series_and_filter(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(
            [rev for rev in self.expected
             if '16' in rev['series'] and rev['arch'] == 'amd64'],
            self.client.get_snap_history(
                'basic', series='16', arch='amd64'))

    def test_get_snap_history_filter_by_unknown_series(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(storeapi.errors.SnapNotFoundError) as e:
            self.client.get_snap_history('basic', series='12')
        self.assertEqual(
            "Snap 'basic' was not found in '12' series.",
            str(e.exception))

    def test_get_snap_history_filter_by_unknown_arch(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(storeapi.errors.SnapNotFoundError) as e:
            self.client.get_snap_history('basic', arch='somearch')
        self.assertEqual(
            "Snap 'basic' for 'somearch' was not found in '16' series.",
            str(e.exception))

    def test_get_snap_history_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.assertEqual(self.expected, self.client.get_snap_history('basic'))
        self.assertFalse(self.fake_store.needs_refresh)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.SCAClient, 'get')
    def test_get_snap_history_server_error(
            self, mock_sca_get, mock_account_info):
        mock_account_info.return_value = {
            'snaps': {
                '16': {
                    'basic': {
                        'snap-id': 'my_snap_id'}}}}

        mock_sca_get.return_value = mock.Mock(
            ok=False, status_code=500, reason='Server error', json=lambda: {})

        self.client.login('dummy', 'test correct password')
        with self.assertRaises(storeapi.errors.StoreSnapHistoryError) as e:
            self.client.get_snap_history('basic')
        self.assertEqual(
            "Error fetching history of snap id 'my_snap_id' for 'any arch' "
            "in '16' series: 500 Server error.",
            str(e.exception))


class GetSnapStatusTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()
        self.expected = {
            'i386': [
                {
                    'info': 'none',
                    'channel': 'stable'
                },
                {
                    'info': 'none',
                    'channel': 'beta'
                },
                {
                    'info': 'specific',
                    'version': '1.0-i386',
                    'channel': 'edge',
                    'revision': 3
                },
            ],
            'amd64': [
                {
                    'info': 'specific',
                    'version': '1.0-amd64',
                    'channel': 'stable',
                    'revision': 2
                },
                {
                    'info': 'specific',
                    'version': '1.1-amd64',
                    'channel': 'beta',
                    'revision': 4
                },
                {
                    'info': 'tracking',
                    'channel': 'edge'
                },
            ],
        }

    def test_get_snap_status_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.get_snap_status('basic')

    def test_get_snap_status_successfully(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(self.expected, self.client.get_snap_status('basic'))

    def test_get_snap_status_filter_by_series(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(
            self.expected,
            self.client.get_snap_status('basic', series='16'))

    def test_get_snap_status_filter_by_arch(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(
            {'amd64': self.expected['amd64']},
            self.client.get_snap_status('basic', arch='amd64'))

    def test_get_snap_status_filter_by_series_and_filter(self):
        self.client.login('dummy', 'test correct password')
        self.assertEqual(
            {'amd64': self.expected['amd64']},
            self.client.get_snap_status(
                'basic', series='16', arch='amd64'))

    def test_get_snap_status_filter_by_unknown_series(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(storeapi.errors.SnapNotFoundError) as e:
            self.client.get_snap_status('basic', series='12')
        self.assertEqual(
            "Snap 'basic' was not found in '12' series.",
            str(e.exception))

    def test_get_snap_status_filter_by_unknown_arch(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(storeapi.errors.SnapNotFoundError) as e:
            self.client.get_snap_status('basic', arch='somearch')
        self.assertEqual(
            "Snap 'basic' for 'somearch' was not found in '16' series.",
            str(e.exception))

    def test_get_snap_status_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.assertEqual(self.expected, self.client.get_snap_status('basic'))
        self.assertFalse(self.fake_store.needs_refresh)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.SCAClient, 'get')
    def test_get_snap_status_server_error(
            self, mock_sca_get, mock_account_info):
        mock_account_info.return_value = {
            'snaps': {'16': {'basic': {'snap-id': 'my_snap_id'}}}}

        mock_sca_get.return_value = mock.Mock(
            ok=False, status_code=500, reason='Server error', json=lambda: {})

        self.client.login('dummy', 'test correct password')
        with self.assertRaises(storeapi.errors.StoreSnapStatusError) as e:
            self.client.get_snap_status('basic')
        self.assertEqual(
            "Error fetching status of snap id 'my_snap_id' for 'any arch' "
            "in '16' series: 500 Server error.",
            str(e.exception))


class SignDeveloperAgreementTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_sign_dev_agreement_success(self):
        self.client.login('dummy', 'test correct password')
        response = {
            "content": {
                "latest_tos_accepted": True,
                "tos_url": "http://fake-url.com",
                "latest_tos_date": "2000-01-01",
                "accepted_tos_date": "2010-10-10"
                }
            }
        self.assertEqual(
            response,
            self.client.sign_developer_agreement(latest_tos_accepted=True))

    def test_sign_dev_agreement_exception(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.DeveloperAgreementSignError) as raised:
            self.client.sign_developer_agreement(False)
        self.assertIn(
            'There was an error while signing developer agreement.\n'
            'Reason: \'Bad Request\'\n',
            str(raised.exception))

    def test_sign_dev_agreement_exception_store_down(self):
        self.useFixture(fixtures.EnvironmentVariable('STORE_DOWN', '1'))
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.DeveloperAgreementSignError) as raised:
            self.client.sign_developer_agreement(latest_tos_accepted=True)
        self.assertEqual(
            str(raised.exception),
            'There was an error while signing developer agreement.\n'
            'Reason: \'Internal Server Error\'\n'
            'Text: \'Broken\'')
