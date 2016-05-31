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
from unittest import mock

import fixtures
import requests

from snapcraft import (
    storeapi,
    tests
)
from snapcraft.storeapi import constants
from snapcraft.tests import fixture_setup


class LoginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()

    def test_login_successful(self):
        result = self.client.login(
            'dummy email',
            'test correct password',
            'dummy token')
        self.assertTrue(result['success'])
        # Assert that the result includes the response body.
        self.assertIn('success', result['body'])
        self.assertTrue(result['body']['success'])

    def test_login_successful_with_one_time_password(self):
        result = self.client.login(
            'dummy email',
            'test correct password',
            'dummy token',
            'test correct one-time password')
        self.assertTrue(result['success'])
        # Assert that the result includes the response body.
        self.assertIn('success', result['body'])
        self.assertTrue(result['body']['success'])

    def test_failed_login_with_wrong_password(self):
        result = self.client.login(
            'dummy email',
            'wrong password',
            'dummy token')
        self.assertFalse(result['success'])
        # Assert that the result includes the response body.
        self.assertEqual(result['body'], {'success': False})

    def test_failed_login_with_wrong_one_time_password(self):
        result = self.client.login(
            'dummy email',
            'test correct password',
            'dummy token',
            'wrong one-time password')
        self.assertFalse(result['success'])
        # Assert that the result includes the response body.
        self.assertEqual(result['body'], {'success': False})


class SSOClientLoginTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.client = storeapi.StoreClient()
        self.email = 'user@domain.com'
        self.password = 'password'
        self.token_name = 'token-name'

        patcher = mock.patch('ssoclient.v2.http.requests.Session.request')
        self.mock_request = patcher.start()
        self.addCleanup(patcher.stop)
        self.token_data = {
            'consumer_key': 'consumer-key',
            'consumer_secret': 'consumer-secret',
            'token_key': 'token-key',
            'token_secret': 'token-secret',
        }
        response = self.make_response(status_code=201, reason='CREATED',
                                      data=self.token_data)
        self.mock_request.return_value = response

    def make_response(self, status_code=200, reason='OK', data=None):
        data = data or {}
        response = requests.Response()
        response.status_code = status_code
        response.reason = reason
        response._content = json.dumps(data).encode('utf-8')
        return response

    def assert_login_request(self):
        data = {
            'email': self.email,
            'password': self.password,
            'token_name': self.token_name
        }
        self.mock_request.assert_called_once_with(
            'POST', constants.UBUNTU_SSO_API_ROOT_URL + 'tokens/oauth',
            data=json.dumps(data),
            json=None, headers={'Content-Type': 'application/json'}
        )

    def test_login_request(self):
        result = self.client.login(self.email, self.password, self.token_name)
        expected = {'success': True, 'body': self.token_data}
        self.assertEqual(result, expected)
        self.assert_login_request()

    def test_login_unsuccessful_unexpected_error(self):
        error_data = {
            'message': 'Error during login.',
            'code': 'UNEXPECTED_ERROR_CODE',
            'extra': {},
        }
        response = self.make_response(
            status_code=401, reason='UNAUTHORISED', data=error_data)
        self.mock_request.return_value = response

        result = self.client.login(self.email, self.password, self.token_name)
        expected = {'success': False, 'body': error_data}
        self.assertEqual(result, expected)


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
        with self.assertRaises(storeapi.InvalidCredentialsError):
            self.client.download('dummy', 'dummy', 'dummy')

    def test_download_unexisting_snap_raises_exception(self):
        self.client.login('dummy', 'test correct password', 'dummy')
        with self.assertRaises(storeapi.SnapNotFoundError) as e:
            self.client.download(
                'unexisting-snap', 'test-channel', 'dummy', 'test-arch')
        self.assertEqual(
            'The "unexisting-snap" for test-arch was not found in '
            'test-channel.',
            str(e.exception))

    def test_download_snap(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login('dummy', 'test correct password', 'dummy')
        download_path = os.path.join(self.path, 'test-snap.snap')
        self.client.download(
            'test-snap', 'test-channel', download_path)
        self.assertIn(
            'Successfully downloaded test-snap at {}'.format(download_path),
            self.fake_logger.output)

    def test_download_already_downloaded_snap(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login('dummy', 'test correct password', 'dummy')
        download_path = os.path.join(self.path, 'test-snap.snap')
        # download first time.
        self.client.download(
            'test-snap', 'test-channel', download_path)
        with mock.patch.object(storeapi.SnapIndexClient, 'get') as mock_get:
            # download again.
            self.client.download(
                'test-snap', 'test-channel', download_path)
        self.assertFalse(mock_get.called)
        self.assertIn(
            'Already downloaded test-snap at {}'.format(download_path),
            self.fake_logger.output)

    def test_download_on_sha_mismatch(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login('dummy', 'test correct password', 'dummy')
        download_path = os.path.join(self.path, 'test-snap.snap')
        # Write a wrong file in the download path.
        open(download_path, 'w').close()
        self.client.download(
            'test-snap', 'test-channel', download_path)
        self.assertIn(
            'Successfully downloaded test-snap at {}'.format(download_path),
            self.fake_logger.output)

    def test_download_with_hash_mismatch_raises_exception(self):
        self.client.login('dummy', 'test correct password', 'dummy')
        download_path = os.path.join(self.path, 'test-snap.snap')
        with self.assertRaises(storeapi.SHAMismatchError):
            self.client.download(
                'test-snap-with-wrong-sha', 'test-channel', download_path)
