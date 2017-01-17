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
from textwrap import dedent
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


class ListKeysTestCase(tests.TestCase):

    scenarios = [
        ('list-keys', {'command_name': 'list-keys'}),
        ('keys alias', {'command_name': 'keys'}),
    ]

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_list_keys_snapd_not_installed(self, mock_installed,
                                           mock_check_output):
        mock_installed.return_value = False

        raised = self.assertRaises(
            SystemExit,
            main, [self.command_name])

        mock_installed.assert_called_with('snapd')
        self.assertEqual(0, mock_check_output.call_count)
        self.assertEqual(1, raised.code)
        self.assertIn(
            'The snapd package is not installed.', self.fake_logger.output)

    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_list_keys_without_login(self, mock_installed, mock_check_output):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output

        raised = self.assertRaises(
            SystemExit,
            main,
            [self.command_name])

        self.assertEqual(1, raised.code)
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch('subprocess.check_output')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_list_keys_successfully(self, mock_installed, mock_check_output,
                                    mock_get_account_information):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {
            'account_id': 'abcd',
            'account_keys': [
                {
                    'name': 'default',
                    'public-key-sha3-384': (
                        get_sample_key('default')['sha3-384']),
                },
            ],
        }

        main([self.command_name])

        expected_output = dedent('''\
                Name     SHA3-384 fingerprint
            *   default  {default_sha3_384}
            -   another  {another_sha3_384}  (not registered)
            ''').format(
                default_sha3_384=get_sample_key('default')['sha3-384'],
                another_sha3_384=get_sample_key('another')['sha3-384'])
        self.assertIn(expected_output, self.fake_terminal.getvalue())
