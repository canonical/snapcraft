# -*- coding: utf-8 -*-
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
from __future__ import absolute_import, unicode_literals
import json
from unittest import TestCase
from unittest.mock import patch

from requests import Response

from snapcraft.storeapi._login import login
from snapcraft.storeapi.constants import (
    UBUNTU_SSO_API_ROOT_URL,
)


class LoginAPITestCase(TestCase):

    def setUp(self):
        super(LoginAPITestCase, self).setUp()
        self.email = 'user@domain.com'
        self.password = 'password'
        self.token_name = 'token-name'

        # setup patches
        mock_environ = {
            'UBUNTU_SSO_API_ROOT_URL': UBUNTU_SSO_API_ROOT_URL,
        }
        patcher = patch('snapcraft.storeapi._login.os.environ', mock_environ)
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('ssoclient.v2.http.requests.Session.request')
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
        response = Response()
        response.status_code = status_code
        response.reason = reason
        response._content = json.dumps(data).encode('utf-8')
        return response

    def assert_login_request(self, otp='', token_name=None):
        if token_name is None:
            token_name = self.token_name
        data = {
            'email': self.email,
            'password': self.password,
            'token_name': token_name
        }
        if otp:
            data['otp'] = otp
        self.mock_request.assert_called_once_with(
            'POST', UBUNTU_SSO_API_ROOT_URL + 'tokens/oauth',
            data=json.dumps(data),
            json=None, headers={'Content-Type': 'application/json'}
        )

    def test_login_successful(self):
        result = login(self.email, self.password, self.token_name)
        expected = {'success': True, 'body': self.token_data}
        self.assertEqual(result, expected)
        self.assert_login_request()

    def test_default_token_name(self):
        result = login(self.email, self.password, self.token_name)
        expected = {'success': True, 'body': self.token_data}
        self.assertEqual(result, expected)
        self.assert_login_request()

    def test_custom_token_name(self):
        result = login(self.email, self.password, token_name='my-token')
        expected = {'success': True, 'body': self.token_data}
        self.assertEqual(result, expected)
        self.assert_login_request(token_name='my-token')

    def test_login_with_otp(self):
        result = login(self.email, self.password, self.token_name,
                       otp='123456')
        expected = {'success': True, 'body': self.token_data}
        self.assertEqual(result, expected)
        self.assert_login_request(otp='123456')

    def test_login_with_empty_otp_omits_it_from_request(self):
        result = login(self.email, self.password, self.token_name, otp='')
        expected = {'success': True, 'body': self.token_data}
        self.assertEqual(result, expected)
        self.assert_login_request()

    def test_login_unsuccessful_api_exception(self):
        error_data = {
            'message': 'Error during login.',
            'code': 'INVALID_CREDENTIALS',
            'extra': {},
        }
        response = self.make_response(
            status_code=401, reason='UNAUTHORISED', data=error_data)
        self.mock_request.return_value = response

        result = login(self.email, self.password, self.token_name)
        expected = {'success': False, 'body': error_data}
        self.assertEqual(result, expected)

    def test_login_unsuccessful_unexpected_error(self):
        error_data = {
            'message': 'Error during login.',
            'code': 'UNEXPECTED_ERROR_CODE',
            'extra': {},
        }
        response = self.make_response(
            status_code=401, reason='UNAUTHORISED', data=error_data)
        self.mock_request.return_value = response

        result = login(self.email, self.password, self.token_name)
        expected = {'success': False, 'body': error_data}
        self.assertEqual(result, expected)
