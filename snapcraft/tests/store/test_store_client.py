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
import subprocess
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

    def test_failed_login_with_wrong_one_time_password(self):
        with self.assertRaises(errors.StoreAuthenticationError):
            self.client.login(
                'dummy email',
                'test correct password',
                'wrong one-time password')

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


class RegisterTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_register_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.register('dummy')

    def test_register_name_successfully(self):
        self.client.login('dummy', 'test correct password')
        # No exception will be raised if this is succesful
        self.client.register('test-good-snap-name')

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
            "name at 'https://myapps.com/register-name-dispute/'")

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
            "name at 'https://myapps.com/register-name-dispute/'")

    def test_registering_too_fast_in_a_row(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreRegistrationError) as raised:
            self.client.register('test-too-fast')
        self.assertEqual(
            str(raised.exception),
            'You must wait 3 minutes before trying to register your '
            'next snap.')

    def test_unhandled_registration_error_path(self):
        self.client.login('dummy', 'test correct password')
        with self.assertRaises(errors.StoreRegistrationError) as raised:
            self.client.register('snap-name-no-clear-error')
        self.assertEqual(str(raised.exception), 'Registration failed.')


class UploadTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()
        self.snap_path = os.path.join(
            os.path.dirname(tests.__file__), 'data',
            'test-snap.snap')
        patcher = mock.patch(
            'snapcraft.storeapi._upload.ProgressBar',
            new=tests.SilentProgressBar)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_upload_unexisting_snap_raises_exception(self):
        with self.assertRaises(subprocess.CalledProcessError):
            self.client.upload('unexisting.snap')

    def test_upload_without_login_raises_exception(self):
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.upload(self.snap_path)

    def test_upload_snap(self):
        self.client.login('dummy', 'test correct password')
        result = self.client.upload(self.snap_path)
        self.assertTrue(result['success'])
        self.assertEqual('test-application-url', result['application_url'])
        self.assertEqual('test-revision', result['revision'])

    def test_upload_with_invalid_credentials_raises_exception(self):
        conf = config.Config()
        conf.set('macaroon', 'inval"id')
        conf.save()
        with self.assertRaises(errors.InvalidCredentialsError):
            self.client.upload(self.snap_path)


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
