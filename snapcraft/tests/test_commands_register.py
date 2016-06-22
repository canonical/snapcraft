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
        with self.assertRaises(docopt.DocoptExit) as raised:
            main(['register'])

        self.assertTrue('Usage:' in str(raised.exception))

    def test_register_without_login_must_raise_exception(self):
        with self.assertRaises(SystemExit):
            main(['register', 'dummy'])
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    def test_register_name_successfully(self):
        with mock.patch.object(
                storeapi.SCAClient, 'register') as mock_register:
            mock_register.return_value.ok = True
            main(['register', 'test-snap'])

        self.assertEqual(
            'Registering test-snap.\n'
            "Congratulations! You're now the publisher for 'test-snap'.\n",
            self.fake_logger.output)

        mock_register.assert_called_once_with('test-snap', '16')

    def test_registration_failed(self):
        with mock.patch.object(
                storeapi.SCAClient, 'register') as mock_register:
            mock_register.return_value.ok = False
            with self.assertRaises(SystemExit):
                main(['register', 'test-snap'])

        self.assertEqual(
            'Registering test-snap.\n'
            'Registration failed.\n',
            self.fake_logger.output)
