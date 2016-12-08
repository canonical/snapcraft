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
from snapcraft.tests import fixture_setup


class StatusCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.expected = {
            'i386': [
                {
                    'info': 'none',
                    'channel': 'stable'
                },
                {
                    'info': 'none',
                    'channel': 'beta'
                },
                {
                    'info': 'specific',
                    'version': '1.0-i386',
                    'channel': 'edge',
                    'revision': 3
                },
            ],
            'amd64': [
                {
                    'info': 'specific',
                    'version': '1.0-amd64',
                    'channel': 'stable',
                    'revision': 2
                },
                {
                    'info': 'specific',
                    'version': '1.1-amd64',
                    'channel': 'beta',
                    'revision': 4
                },
                {
                    'info': 'tracking',
                    'channel': 'edge'
                },
            ],
        }

    def test_status_without_snap_raises_exception(self):
        raised = self.assertRaises(
            docopt.DocoptExit,
            main, ['status'])

        self.assertIn('Usage:', str(raised))

    def test_status_with_no_permissions(self):
        self.assertRaises(
            SystemExit,
            main, ['status', 'snap-test'])

        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?',
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status_with_3rd_party_snap(self, mock_account_api):
        mock_account_api.return_value = {}

        self.assertRaises(
            SystemExit,
            main, ['status', 'snap-test'])

        self.assertIn(
            "Snap 'snap-test' was not found in '16' series.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status_with_3rd_party_snap_by_arch(self, mock_account_api):
        mock_account_api.return_value = {}

        self.assertRaises(
            SystemExit,
            main, ['status', 'snap-test', '--arch=arm64'])

        self.assertIn(
            "Snap 'snap-test' for 'arm64' was not found in '16' series.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status_with_3rd_party_snap_by_series(self, mock_account_api):
        mock_account_api.return_value = {}

        self.assertRaises(
            SystemExit,
            main, ['status', 'snap-test', '--series=15'])

        self.assertIn(
            "Snap 'snap-test' was not found in '15' series.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'snap_status')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status_by_unknown_arch(self, mock_account_api, mock_status):
        mock_status.return_value = {}

        self.assertRaises(
            SystemExit,
            main, ['status', 'snap-test', '--arch=some-arch'])

        self.assertIn(
            "Snap 'snap-test' for 'some-arch' was not found in '16' series.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'snap_status')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status_by_unknown_series(self, mock_account_api, mock_status):
        mock_status.return_value = {}

        self.assertRaises(
            SystemExit,
            main, ['status', 'snap-test', '--series=some-series'])

        self.assertIn(
            "Snap 'snap-test' was not found in 'some-series' series.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_snap_status')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status(self, mock_account_api, mock_status):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        mock_status.return_value = self.expected

        main(['status', 'snap-test'])

        mock_status.assert_called_once_with('snap-test', '16', None)

        terminal_output = fake_terminal.getvalue()
        expected_output = [
            'Arch    Channel    Version    Revision',
            'amd64   stable     1.0-amd64  2',
            '        beta       1.1-amd64  4',
            '        edge       ^          ^',
            'i386    stable     -          -',
            '        beta       -          -',
            '        edge       1.0-i386   3']
        self.assertEqual(expected_output, terminal_output.splitlines())

    @mock.patch.object(storeapi.StoreClient, 'get_snap_status')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status_by_arch(self, mock_account_api, mock_status):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)
        mock_status.return_value = {'i386': self.expected['i386']}

        main(['status', 'snap-test', '--arch=i386'])

        mock_status.assert_called_once_with('snap-test', '16', 'i386')

        terminal_output = fake_terminal.getvalue()
        expected_output = [
            'Arch    Channel    Version    Revision',
            'i386    stable     -          -',
            '        beta       -          -',
            '        edge       1.0-i386   3']
        self.assertEqual(expected_output, terminal_output.splitlines())

    @mock.patch.object(storeapi.StoreClient, 'get_snap_status')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_status_by_series(self, mock_account_api, mock_status):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)
        mock_status.return_value = self.expected

        main(['status', 'snap-test', '--series=16'])

        mock_status.assert_called_once_with('snap-test', '16', None)

        terminal_output = fake_terminal.getvalue()
        expected_output = [
            'Arch    Channel    Version    Revision',
            'amd64   stable     1.0-amd64  2',
            '        beta       1.1-amd64  4',
            '        edge       ^          ^',
            'i386    stable     -          -',
            '        beta       -          -',
            '        edge       1.0-i386   3']
        self.assertEqual(expected_output, terminal_output.splitlines())
