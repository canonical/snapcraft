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
from unittest import mock
import re

from testtools.matchers import Contains, Equals, MatchesRegex, Not

from snapcraft import storeapi
from . import CommandBaseTestCase


class ExportLoginCommandTestCase(CommandBaseTestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('builtins.input')
        self.mock_input = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.print')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('getpass.getpass')
        patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch.object(storeapi._sca_client.SCAClient,
                       'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch.object(storeapi.StoreClient, 'acl')
    def test_successful_export(
            self, mock_acl, mock_login, mock_get_account_information):
        self.mock_input.return_value = 'user@example.com'
        mock_acl.return_value = {
            'snap_ids': None,
            'channels': None,
            'permissions': None,
        }

        result = self.run_command(['export-login', 'exported'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            storeapi.constants.TWO_FACTOR_WARNING))
        self.assertThat(
            result.output, Contains('Login successfully exported'))
        self.assertThat(
            result.output, MatchesRegex(
                r'.*snaps:.*?No restriction', re.DOTALL))
        self.assertThat(
            result.output, MatchesRegex(
                r'.*channels:.*?No restriction', re.DOTALL))
        self.assertThat(
            result.output, MatchesRegex(
                r'.*permissions:.*?No restriction', re.DOTALL))

        self.mock_input.assert_called_once_with('Email: ')
        mock_login.assert_called_once_with(
            'user@example.com', mock.ANY, acls=None, packages=None,
            channels=None, save=False, config_fd=None)
        mock_acl.assert_called_once_with()

    @mock.patch.object(storeapi._sca_client.SCAClient,
                       'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch.object(storeapi.StoreClient, 'acl')
    def test_successful_login_with_2fa(
            self, mock_acl, mock_login, mock_get_account_information):
        self.mock_input.side_effect = ('user@example.com', '123456')
        mock_login.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(),
            None]
        mock_acl.return_value = {
            'snap_ids': None,
            'channels':['edge'],
            'permissions': None,
        }

        result = self.run_command(['export-login', 'exported'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Not(Contains(
            storeapi.constants.TWO_FACTOR_WARNING)))
        self.assertThat(
            result.output, Contains('Login successfully exported'))
        self.assertThat(
            result.output, MatchesRegex(
                r'.*snaps:.*?No restriction', re.DOTALL))
        self.assertThat(
            result.output, MatchesRegex(
                r".*channels:.*?['edge123']", re.DOTALL))
        self.assertThat(
            result.output, MatchesRegex(
                r'.*permissions:.*?No restriction', re.DOTALL))

        self.assertThat(self.mock_input.call_count, Equals(2))
        self.mock_input.assert_has_calls([
            mock.call('Email: '), mock.call('Second-factor auth: ')])
        self.assertThat(mock_login.call_count, Equals(2))
        mock_login.assert_has_calls([
            mock.call(
                'user@example.com', mock.ANY, acls=None, packages=None,
                channels=None, save=False, config_fd=None),
            mock.call(
                'user@example.com', mock.ANY, one_time_password='123456',
                acls=None, packages=None, channels=None, save=False,
                config_fd=None)])

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_invalid_credentials(self, mock_login):
        mock_login.side_effect = storeapi.errors.InvalidCredentialsError(
            'error')

        result = self.run_command(['export-login', 'exported'])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(
            storeapi.constants.INVALID_CREDENTIALS))
        self.assertThat(result.output, Contains('Login failed.'))

def _new_snap(store: storeapi.StoreClient) -> str:
    acl = store.acl()
    snap_names = [ ]

    if not(snap_id in acl['snap_ids']):
	  
       snap_names.append('Heesen')
    else:
        for snap_id in acl['snap_ids']:
            snap_names.append(store.get_snap_name_for_id(snap_id))
	    
    acl['snap_names'] = snap_names

         
    @mock.patch.object(storeapi._sca_client.SCAClient,
                       'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch.object(storeapi.StoreClient, 'acl')
    def test_successful_export(
            self, mock_acl, mock_login, mock_get_account_information):
        self.mock_input.return_value = 'user@example.com'
        mock_acl.return_value = {
            'snap_ids':['edge123'],
            'channels': None,
            'permissions': None,
        }

        result = self.run_command(['export-login', 'exported'])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(
            storeapi.constants.TWO_FACTOR_WARNING))
        self.assertThat(
            result.output, Contains('Login successfully exported'))
        self.assertThat(
            result.output, MatchesRegex(
                r".*snaps:.*?['edge123']", re.DOTALL))
        self.assertThat(
            result.output, MatchesRegex(
                r'.*channels:.*?No restriction', re.DOTALL))
        self.assertThat(
            result.output, MatchesRegex(
                r'.*permissions:.*?No restriction', re.DOTALL))

        self.mock_input.assert_called_once_with('Email: ')
        mock_login.assert_called_once_with(
            'user@example.com', mock.ANY, acls=None, packages=None,
            channels=None, save=False, config_fd=None)
        mock_acl.assert_called_once_with()
