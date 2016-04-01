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
from unittest import mock

import fixtures

from snapcraft.main import main
from snapcraft import tests


class LoginCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Ensure SNAPCRAFT_WITH_MACAROONS is not defined
        patcher = mock.patch.dict(os.environ, {})
        patcher.start()
        self.addCleanup(patcher.stop)
        try:
            del os.environ['SNAPCRAFT_WITH_MACAROONS']
        except KeyError:
            pass

        patcher = mock.patch('builtins.input')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.print')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('getpass.getpass')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.config.save_config')
        self.mock_save = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.storeapi.login')
        self.mock_login = patcher.start()
        self.addCleanup(patcher.stop)

    def test_successful_login_saves_config(self):
        response = {
            'success': True,
            'body': {
                'consumer_key': 'consumer_key',
                'consumer_secret': 'consumer_secret',
                'token_key': 'token_key',
                'token_secret': 'token_secret',
            }
        }
        self.mock_login.return_value = response

        main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login successful.\n',
            self.fake_logger.output)
        self.mock_save.assert_called_once_with(response['body'])

    def test_failed_login_does_not_save_config(self):
        response = {
            'success': False,
            'body': {},
        }
        self.mock_login.return_value = response

        main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login failed.\n',
            self.fake_logger.output)
        self.assertFalse(self.mock_save.called)


class LoginWithMacaroonsCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        # Ensure SNAPCRAFT_WITH_MACAROONS is properly set
        patcher = mock.patch.dict(os.environ, SNAPCRAFT_WITH_MACAROONS='1')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.input')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.print')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('getpass.getpass')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.config.save_config')
        self.mock_save = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            'snapcraft.storeapi._macaroon_login.get_root_macaroon')
        self.mock_root_macaroon = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch(
            'snapcraft.storeapi._macaroon_login.get_discharge_macaroon')
        self.mock_discharge_macaroon = patcher.start()
        self.addCleanup(patcher.stop)

    def test_successful_login_saves_config(self):
        self.mock_root_macaroon.return_value = ('root', None)
        self.mock_discharge_macaroon.return_value = ('discharge', None)

        main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login successful.\n',
            self.fake_logger.output)
        self.mock_save.assert_called_once_with(
            dict(root_macaroon='root', discharge_macaroon='discharge'))

    def test_failed_root_macaroon_does_not_save_config(self):
        self.mock_root_macaroon.return_value = (None, 'Failed')

        main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login failed.\n',
            self.fake_logger.output)
        self.assertFalse(self.mock_save.called)

    def test_failed_disacharge_macaroon_does_not_save_config(self):
        response = ('root', None)
        self.mock_root_macaroon.return_value = response
        response = (None, 'Failed')
        self.mock_discharge_macaroon.return_value = response

        main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login failed.\n',
            self.fake_logger.output)
        self.assertFalse(self.mock_save.called)
