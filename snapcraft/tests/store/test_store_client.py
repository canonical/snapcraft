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
from unittest import mock

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
