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

import docopt
import fixtures
from simplejson.scanner import JSONDecodeError

from snapcraft import (
    storeapi,
    tests
)
from snapcraft.main import main


class RegisterTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    def test_register_without_name_must_raise_exception(self):
        raised = self.assertRaises(
            docopt.DocoptExit,
            main, ['register'])

        self.assertTrue('Usage:' in str(raised))

    def test_register_without_login_must_raise_exception(self):
        raised = self.assertRaises(
            SystemExit,
            main, ['register', 'dummy'])

        self.assertEqual(1, raised.code)
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    def test_register_name_successfully(self):
        with mock.patch.object(
                storeapi.SCAClient, 'register') as mock_register:
            main(['register', 'test-snap'])

        self.assertEqual(
            'Registering test-snap.\n'
            "Congratulations! You're now the publisher for 'test-snap'.\n",
            self.fake_logger.output)

        mock_register.assert_called_once_with('test-snap', False, '16')

    def test_register_private_name_successfully(self):
        with mock.patch.object(
                storeapi.SCAClient, 'register') as mock_register:
            main(['register', 'test-snap', '--private'])

        self.assertEqual(
            'Registering test-snap.\n'
            "Congratulations! You're now the publisher for 'test-snap'.\n",
            self.fake_logger.output)

        mock_register.assert_called_once_with('test-snap', True, '16')

    def test_registration_failed(self):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError('mock-fail', 'doc', 1)
        with mock.patch.object(
                storeapi.SCAClient, 'register') as mock_register:
            mock_register.side_effect = storeapi.errors.StoreRegistrationError(
                'test-snap', response)
            raised = self.assertRaises(
                SystemExit,
                main, ['register', 'test-snap'])

        self.assertEqual(1, raised.code)
        self.assertEqual(
            'Registering test-snap.\n'
            'Registration failed.\n',
            self.fake_logger.output)
