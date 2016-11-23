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
from simplejson.scanner import JSONDecodeError

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

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_successful_login(
            self, mock_login, mock_get_account_information):
        self.mock_input.return_value = 'user@example.com'

        # no exception raised.
        main(['login'])

        self.mock_input.assert_called_once_with('Email: ')
        mock_login.assert_called_once_with(
            'user@example.com', mock.ANY, acls=None, packages=None,
            channels=None, save=True)
        self.assertEqual(
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            'Login successful.\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_successful_login_with_2fa(
            self, mock_login, mock_get_account_information):
        self.mock_input.side_effect = ('user@example.com', '123456')
        mock_login.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(),
            None]

        # no exception raised.
        main(['login'])

        self.assertEqual(2, self.mock_input.call_count)
        self.mock_input.assert_has_calls([
            mock.call('Email: '), mock.call('Second-factor auth: ')])
        self.assertEqual(2, mock_login.call_count)
        mock_login.assert_has_calls([
            mock.call(
                'user@example.com', mock.ANY, acls=None, packages=None,
                channels=None, save=True),
            mock.call(
                'user@example.com', mock.ANY, one_time_password='123456',
                acls=None, packages=None, channels=None, save=True)])
        self.assertEqual('Login successful.\n', self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_invalid_credentials(self, mock_login):
        mock_login.side_effect = storeapi.errors.InvalidCredentialsError(
            'error')

        main(['login'])

        self.assertEqual(
            storeapi.constants.INVALID_CREDENTIALS +
            '\nLogin failed.\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_store_authentication_error(self, mock_login):
        mock_login.side_effect = storeapi.errors.StoreAuthenticationError(
            'error')

        main(['login'])

        self.assertEqual(
            storeapi.constants.AUTHENTICATION_ERROR +
            '\nLogin failed.\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_store_account_info_error(self, mock_login):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError('mock-fail', 'doc', 1)
        response.status_code = 500
        response.reason = 'Internal Server Error'
        mock_login.side_effect = storeapi.errors.StoreAccountInformationError(
            response)

        main(['login'])

        self.assertEqual(
            storeapi.constants.ACCOUNT_INFORMATION_ERROR +
            '\nLogin failed.\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_dev_agreement_error(
            self, mock_login, mock_acc_info):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {'error_list': [{
            'message': storeapi.constants.MISSING_AGREEMENT,
            'extra': {
                'url': 'http://fake-url.com',
                'api': 'fake-api'}}]}
        response.json.return_value = content
        account_info_exception = storeapi.errors.StoreAccountInformationError(
            response)
        mock_acc_info.side_effect = account_info_exception

        main(['login'])

        self.assertEqual((
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            storeapi.constants.AGREEMENT_ERROR +
            '\nLogin failed.\n'),
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_dev_namespace_error(
            self, mock_login, mock_acc_info):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_NAMESPACE
        content = {'error_list': [{
            'message': storeapi.constants.MISSING_NAMESPACE,
            'extra': {
                'url': 'http://fake-url.com',
                'api': 'fake-api'}}]}
        response.json.return_value = content
        account_info_exception = storeapi.errors.StoreAccountInformationError(
            response)
        mock_acc_info.side_effect = account_info_exception

        main(['login'])

        self.assertEqual((
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            storeapi.constants.NAMESPACE_ERROR.format('http://fake-url.com') +
            '\nLogin failed.\n'),
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_unexpected_account_error(
            self, mock_login, mock_acc_info):
        # Test to simulate get_account_info raising unexpected errors.
        response = mock.Mock()
        response.status_code = 500
        response.reason = 'Internal Server Error'
        content = {'error_list': [{
            'message': "Just another error",
            'extra': {
                'url': 'http://fake-url.com',
                'api': 'fake-api'}}]}
        response.json.return_value = content
        side_effect = storeapi.errors.StoreAccountInformationError(response)
        mock_acc_info.side_effect = side_effect

        main(['login'])

        self.assertEqual(
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            storeapi.constants.ACCOUNT_INFORMATION_ERROR +
            '\nLogin failed.\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_dev_agreement_error_with_choice(
            self, mock_login, mock_acc_info):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {'error_list': [{
            'message': storeapi.constants.MISSING_AGREEMENT,
            'extra': {
                'url': 'http://fake-url.com',
                'api': 'fake-api'}}]}
        response.json.return_value = content
        account_info_exception = storeapi.errors.StoreAccountInformationError(
            response)
        mock_acc_info.side_effect = account_info_exception

        main(['login'])
        self.mock_input.assert_called_with(
            storeapi.constants.AGREEMENT_INPUT_MSG.format(
                'http://fake-url.com'))

        self.assertEqual((
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            storeapi.constants.AGREEMENT_ERROR +
            '\nLogin failed.\n'),
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_dev_agreement_error_with_choice_no(
            self, mock_login, mock_acc_info):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {'error_list': [{
            'message': storeapi.constants.MISSING_AGREEMENT,
            'extra': {
                'url': 'http://fake-url.com',
                'api': 'fake-api'}}]}
        response.json.return_value = content
        account_info_exception = storeapi.errors.StoreAccountInformationError(
            response)
        mock_acc_info.side_effect = account_info_exception

        self.mock_input.return_value = 'n'
        main(['login'])

        self.assertEqual((
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            storeapi.constants.AGREEMENT_ERROR +
            '\nLogin failed.\n'),
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_dev_agreement_error_with_choice_yes(
            self, mock_login, mock_acc_info):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {'error_list': [{
            'message': storeapi.constants.MISSING_AGREEMENT,
            'extra': {
                'url': 'http://fake-url.com',
                'api': 'fake-api'}}]}
        response.json.return_value = content
        account_info_exception = storeapi.errors.StoreAccountInformationError(
            response)
        mock_acc_info.side_effect = account_info_exception

        self.mock_input.return_value = 'y'
        main(['login'])

        self.assertEqual((
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            storeapi.constants.AGREEMENT_SIGN_ERROR.format(
                'http://fake-url.com') +
            '\nLogin failed.\n'),
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'sign_developer_agreement')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    def test_failed_login_with_dev_agreement_error_with_sign_success(
            self, mock_login, mock_acc_info, mock_sign_agreement):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {'error_list': [{
            'message': storeapi.constants.MISSING_AGREEMENT,
            'extra': {
                'url': 'http://fake-url.com',
                'api': 'fake-api'}}]}
        response.json.return_value = content
        account_info_exception = storeapi.errors.StoreAccountInformationError(
            response)
        mock_acc_info.side_effect = account_info_exception

        self.mock_input.return_value = 'y'
        main(['login'])

        self.assertEqual((
            storeapi.constants.TWO_FACTOR_WARNING + '\n' +
            storeapi.constants.ACCOUNT_INFORMATION_ERROR +
            '\nLogin failed.\n'),
            self.fake_logger.output)
