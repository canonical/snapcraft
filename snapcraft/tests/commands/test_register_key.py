# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
from simplejson.scanner import JSONDecodeError
from textwrap import dedent
from unittest import mock

from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from . import CommandBaseTestCase, get_sample_key, mock_snap_output


class RegisterKeyTestCase(CommandBaseTestCase):

    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
    def test_register_key_snapd_not_installed(self, mock_installed,
                                              mock_check_output):
        mock_installed.return_value = False

        result = self.run_command(['register-key'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(
            'The snapd package is not installed.'))
        mock_installed.assert_called_with('snapd')
        self.assertEqual(0, mock_check_output.call_count)

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
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

        result = self.run_command(['register-key', 'default'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            'Login successful.'))
        self.assertThat(result.output, Contains(
            'Registering key ...'))
        self.assertThat(result.output, Contains(
            'Done. The key "default" ({}) may be used to sign your '
            'assertions.'.format(get_sample_key('default')['sha3-384'])))
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
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
    def test_register_key_no_keys(self, mock_installed, mock_input,
                                  mock_check_output):
        mock_installed.return_value = True
        mock_check_output.return_value = json.dumps([])

        result = self.run_command(['register-key'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains('You have no usable keys'))
        self.assertEqual(0, mock_input.call_count)

    @mock.patch('subprocess.check_output')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
    def test_register_key_no_keys_null(self, mock_installed, mock_input,
                                       mock_check_output):
        # Some versions of snapd serialise an empty list as "null" rather
        # than "[]".
        mock_installed.return_value = True
        mock_check_output.return_value = json.dumps(None)

        result = self.run_command(['register-key'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains('You have no usable keys.'))
        self.assertEqual(0, mock_input.call_count)

    @mock.patch('subprocess.check_output')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
    def test_register_key_no_keys_with_name(self, mock_installed, mock_input,
                                            mock_check_output):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output

        result = self.run_command(['register-key', 'nonexistent'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(
            'You have no usable key named "nonexistent"'))
        self.assertEqual(0, mock_input.call_count)

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
    def test_register_key_login_failed(self, mock_installed, mock_input,
                                       mock_getpass, mock_check_output,
                                       mock_login,
                                       mock_get_account_information,
                                       mock_register_key):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_login.side_effect = storeapi.errors.StoreAuthenticationError(
            'test')

        result = self.run_command(['register-key', 'default'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(
            'Cannot continue without logging in successfully.\n'))
        self.assertEqual(1, mock_input.call_count)

    @mock.patch('snapcraft._store._login')
    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
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

        result = self.run_command(['register-key', 'default'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(
            'Error fetching account information from store: '
            '500 Internal Server Error'))

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
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

        result = self.run_command(['register-key', 'default'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(
            'Key registration failed: 500 Internal Server Error'))

    @mock.patch.object(storeapi.SCAClient, 'register_key')
    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.Repo.is_package_installed')
    def test_register_key_select_key(self, mock_installed, mock_input,
                                     mock_getpass, mock_check_output,
                                     mock_login, mock_get_account_information,
                                     mock_register_key):
        mock_installed.return_value = True
        mock_input.side_effect = ['x', '2', 'sample.person@canonical.com']
        mock_getpass.return_value = 'secret'
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {'account_id': 'abcd'}

        result = self.run_command(['register-key'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(dedent("""\
            Select a key:

              Number  Name     SHA3-384 fingerprint
                   1  default  {default_sha3_384}
                   2  another  {another_sha3_384}
            """).format(
                default_sha3_384=get_sample_key('default')['sha3-384'],
                another_sha3_384=get_sample_key('another')['sha3-384'])))
        mock_input.assert_has_calls(
            [mock.call('Key number: '), mock.call('Key number: ')])

        self.assertEqual(
            'We strongly recommend enabling multi-factor authentication: '
            'https://help.ubuntu.com/community/SSO/FAQs/2FA\n'
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
