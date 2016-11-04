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
from unittest import mock

import fixtures

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests
)


class LoginCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        patcher = mock.patch('builtins.input')
        self.mock_input = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.print')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('getpass.getpass')
        patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_successful_login(self, mock_login):
        self.mock_input.return_value = 'user@example.com'

        # no exception raised.
        main(['login'])

        self.mock_input.assert_called_once_with('Email: ')
        mock_login.assert_called_once_with(
            'user@example.com', mock.ANY, acls=None, save=True)
        self.assertEqual(
            'We strongly recommend enabling multi-factor authentication: '
            'https://help.ubuntu.com/community/SSO/FAQs/2FA\n'
            'Login successful.\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_successful_login_with_2fa(self, mock_login):
        self.mock_input.side_effect = ('user@example.com', '123456')
        mock_login.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(), None]

        # no exception raised.
        main(['login'])

        self.assertEqual(2, self.mock_input.call_count)
        self.mock_input.assert_has_calls([
            mock.call('Email: '), mock.call('Second-factor auth: ')])
        self.assertEqual(2, mock_login.call_count)
        mock_login.assert_has_calls([
            mock.call('user@example.com', mock.ANY, acls=None, save=True),
            mock.call(
                'user@example.com', mock.ANY, one_time_password='123456',
                acls=None, save=True)])
        self.assertEqual('Login successful.\n', self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_invalid_credentials(self, mock_login):
        mock_login.side_effect = storeapi.errors.InvalidCredentialsError(
            'error')

        main(['login'])

        self.assertEqual('Login failed.\n', self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_store_authentication_error(self, mock_login):
        mock_login.side_effect = storeapi.errors.StoreAuthenticationError(
            'error')

        main(['login'])

        self.assertEqual('Login failed.\n', self.fake_logger.output)
