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

    def test_download_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.download('dummy', 'dummy', 'dummy')

    def test_download_unexisting_snap_raises_exception(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.SnapNotFoundError) as e:
            self.client.download(
                'unexisting-snap', 'test-channel', 'dummy', 'test-arch')
        self.assertEqual(
            'The "unexisting-snap" for test-arch was not found in '
            'test-channel.',
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

    def test_download_with_invalid_credentials_raises_exception(self):
        conf = config.Config()
        conf.set('macaroon', 'inval"id')
        conf.save()
        download_path = os.path.join(self.path, 'test-snap.snap')
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.download(
                'test-snap', 'test-channel', download_path)


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
        self.assertEqual(
            {'account_id': 'abcd', 'account_keys': []},
            self.client.get_account_information())

    def test_get_account_information_refreshes_macaroon(self):
        self.client.login('dummy', 'test correct password')
        self.fake_store.needs_refresh = True
        self.assertEqual(
            {'account_id': 'abcd', 'account_keys': []},
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
