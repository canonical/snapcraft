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
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.print')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('getpass.getpass')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_successful_login(self):
        with mock.patch.object(storeapi.StoreClient, 'login'):
            # no exception raised.
            main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login successful.\n',
            self.fake_logger.output)

    def test_failed_login_with_invalid_credentials(self):
        with mock.patch.object(
                storeapi.StoreClient, 'login') as mock_login:
            mock_login.side_effect = storeapi.errors.InvalidCredentialsError(
                'error')
            main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login failed.\n',
            self.fake_logger.output)

    def test_failed_login_with_store_authentication_error(self):
        with mock.patch.object(
                storeapi.StoreClient, 'login') as mock_login:
            mock_login.side_effect = storeapi.errors.StoreAuthenticationError(
                'error')
            main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login failed.\n',
            self.fake_logger.output)
