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
from textwrap import dedent
from unittest import mock

import fixtures
from simplejson.scanner import JSONDecodeError

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests,
)
from . import (
    get_sample_key,
    mock_snap_output
)


class RegisterKeyTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_snapd_not_installed(self, mock_installed,
                                              mock_check_output):
        mock_installed.return_value = False

        raised = self.assertRaises(
            SystemExit,
            main, ['register-key'])

        mock_installed.assert_called_with('snapd')
        self.assertEqual(0, mock_check_output.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'The snapd package is not installed.', self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_successfully(self, mock_installed, mock_input,
                                       mock_getpass, mock_check_output,
                                       mock_login,
                                       mock_get_account_information,
                                       mock_register_key):
        mock_installed.return_value = True
        mock_input.side_effect = ['sample.person@canonical.com', '123456']
        mock_getpass.return_value = 'secret'
        mock_check_output.side_effect = mock_snap_output
        mock_login.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(), None]
        mock_get_account_information.return_value = {'account_id': 'abcd'}

        main(['register-key', 'default'])

        self.assertEqual(
            'Login successful.\n'
            'Registering key ...\n'
            'Done. The key "default" ({}) may be used to sign your '
            'assertions.\n'.format(get_sample_key('default')['sha3-384']),
            self.fake_logger.output)

        mock_login.assert_called_with(
            'sample.person@canonical.com', 'secret',
            one_time_password='123456', acls=['modify_account_key'],
            packages=None, channels=None, save=False)
        self.assertEqual(1, mock_register_key.call_count)
        expected_assertion = dedent('''\
            type: account-key-request
            account-id: abcd
            name: default
            public-key-sha3-384: {}
            ''').format(get_sample_key('default')['sha3-384'])
        mock_register_key.assert_called_once_with(expected_assertion)

    @mock.patch('subprocess.check_output')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_no_keys(self, mock_installed, mock_input,
                                  mock_check_output):
        mock_installed.return_value = True
        mock_check_output.return_value = json.dumps([])

        raised = self.assertRaises(
            SystemExit,
            main, ['register-key'])

        self.assertEqual(0, mock_input.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn('You have no usable keys.\n', self.fake_logger.output)

    @mock.patch('subprocess.check_output')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_no_keys_null(self, mock_installed, mock_input,
                                       mock_check_output):
        # Some versions of snapd serialise an empty list as "null" rather
        # than "[]".
        mock_installed.return_value = True
        mock_check_output.return_value = json.dumps(None)

        raised = self.assertRaises(
            SystemExit,
            main, ['register-key'])

        self.assertEqual(0, mock_input.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn('You have no usable keys.\n', self.fake_logger.output)

    @mock.patch('subprocess.check_output')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_no_keys_with_name(self, mock_installed, mock_input,
                                            mock_check_output):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output

        raised = self.assertRaises(
            SystemExit,
            main, ['register-key', 'nonexistent'])

        self.assertEqual(0, mock_input.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'You have no usable key named "nonexistent".\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_login_failed(self, mock_installed, mock_input,
                                       mock_getpass, mock_check_output,
                                       mock_login,
                                       mock_get_account_information,
                                       mock_register_key):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_login.side_effect = storeapi.errors.StoreAuthenticationError(
            'test')

        raised = self.assertRaises(
            SystemExit,
            main, ['register-key', 'default'])

        self.assertEqual(0, mock_get_account_information.call_count)
        self.assertEqual(0, mock_register_key.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'Cannot continue without logging in successfully.\n',
            self.fake_logger.output)

    @mock.patch('snapcraft._store._login')
    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_account_info_failed(self, mock_installed, mock_input,
                                              mock_getpass, mock_check_output,
                                              mock_login,
                                              mock_get_account_information,
                                              mock_register_key,
                                              mock__login):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock__login.return_value = True
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError('mock-fail', 'doc', 1)
        response.status_code = 500
        response.reason = 'Internal Server Error'
        mock_get_account_information.side_effect = (
            storeapi.errors.StoreAccountInformationError(response))

        raised = self.assertRaises(
            SystemExit,
            main, ['register-key', 'default'])

        self.assertEqual(1, raised.code)
        self.assertIn(
            'Error fetching account information from store: '
            '500 Internal Server Error\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_failed(self, mock_installed, mock_input,
                                 mock_getpass, mock_check_output, mock_login,
                                 mock_get_account_information,
                                 mock_register_key):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {'account_id': 'abcd'}
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError('mock-fail', 'doc', 1)
        response.status_code = 500
        response.reason = 'Internal Server Error'
        mock_register_key.side_effect = (
            storeapi.errors.StoreKeyRegistrationError(response))

        raised = self.assertRaises(
            SystemExit,
            main, ['register-key', 'default'])

        self.assertEqual(1, raised.code)
        self.assertIn(
            'Key registration failed: 500 Internal Server Error\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_register_key_select_key(self, mock_installed, mock_input,
                                     mock_getpass, mock_check_output,
                                     mock_login, mock_get_account_information,
                                     mock_register_key):
        mock_installed.return_value = True
        mock_input.side_effect = ['x', '2', 'sample.person@canonical.com']
        mock_getpass.return_value = 'secret'
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {'account_id': 'abcd'}

        main(['register-key'])

        expected_output = dedent('''\
            Select a key:

              Number  Name     SHA3-384 fingerprint
                   1  default  {default_sha3_384}
                   2  another  {another_sha3_384}

            ''').format(
                default_sha3_384=get_sample_key('default')['sha3-384'],
                another_sha3_384=get_sample_key('another')['sha3-384'])
        self.assertIn(expected_output, self.fake_terminal.getvalue())
        mock_input.assert_has_calls(
            [mock.call('Key number: '), mock.call('Key number: ')])

        self.assertEqual(
            'We strongly recommend enabling multi-factor authentication: '
            'https://help.ubuntu.com/community/SSO/FAQs/2FA\n'
            'Login successful.\n'
            'Registering key ...\n'
            'Done. The key "another" ({}) may be used to sign your '
            'assertions.\n'.format(get_sample_key('another')['sha3-384']),
            self.fake_logger.output)

        self.assertEqual(1, mock_register_key.call_count)
        expected_assertion = dedent('''\
            type: account-key-request
            account-id: abcd
            name: another
            public-key-sha3-384: {}
            ''').format(get_sample_key('another')['sha3-384'])
        mock_register_key.assert_called_once_with(expected_assertion)
