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
    tests,
)
from . import (
    get_sample_key,
    mock_snap_output
)


class CreateKeyTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    @mock.patch('subprocess.check_call')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_create_key_snapd_not_installed(self, mock_installed,
                                            mock_check_output,
                                            mock_check_call):
        mock_installed.return_value = False

        raised = self.assertRaises(
            SystemExit,
            main, ['create-key'])

        mock_installed.assert_called_with('snapd')
        self.assertEqual(0, mock_check_output.call_count)
        self.assertEqual(0, mock_check_call.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'The snapd package is not installed.', self.fake_logger.output)

    @mock.patch('subprocess.check_call')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_create_key_already_exists(self, mock_installed, mock_check_output,
                                       mock_check_call):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output

        raised = self.assertRaises(
            SystemExit,
            main, ['create-key'])

        self.assertEqual(0, mock_check_call.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'You already have a key named "default".', self.fake_logger.output)

    @mock.patch('subprocess.check_call')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_create_key_already_registered(self, mock_installed,
                                           mock_check_output,
                                           mock_get_account_information,
                                           mock_check_call):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {
            'account_id': 'abcd',
            'account_keys': [
                {
                    'name': 'new-key',
                    'public-key-sha3-384': (
                        get_sample_key('default')['sha3-384']),
                },
            ],
        }

        raised = self.assertRaises(
            SystemExit,
            main, ['create-key', 'new-key'])

        self.assertEqual(0, mock_check_call.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'You have already registered a key named "new-key".',
            self.fake_logger.output)

    @mock.patch('subprocess.check_call')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_create_key_successfully(self, mock_installed, mock_check_output,
                                     mock_get_account_information,
                                     mock_check_call):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {
            'account_id': 'abcd',
            'account_keys': [
                {
                    'name': 'old-key',
                    'public-key-sha3-384': (
                        get_sample_key('default')['sha3-384']),
                },
            ],
        }

        main(['create-key', 'new-key'])

        mock_check_call.assert_called_once_with(
            ['snap', 'create-key', 'new-key'])
        self.assertEqual('', self.fake_logger.output)

    @mock.patch('subprocess.check_call')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_create_key_without_login(self, mock_installed, mock_check_output,
                                      mock_get_account_information,
                                      mock_check_call):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.side_effect = (
            storeapi.errors.InvalidCredentialsError('Test'))

        main(['create-key', 'new-key'])

        mock_check_call.assert_called_once_with(
            ['snap', 'create-key', 'new-key'])
        self.assertEqual('', self.fake_logger.output)
