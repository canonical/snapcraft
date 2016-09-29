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


class HistoryCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.expected = [{
            'series': ['16'],
            'channels': [],
            'version': '2.0.1',
            'timestamp': '2016-09-27T19:23:40.409',
            'current_channels': ['beta', 'edge'],
            'arch': 'i386',
            'revision': 2
        }, {
            'series': ['16'],
            'channels': ['stable', 'edge'],
            'version': '2.0.1-test',
            'timestamp': '2016-09-27T18:38:43.388',
            'current_channels': ['stable', 'candidate', 'beta'],
            'arch': 'amd64',
            'revision': 1,
        }]

    def test_history_without_snap_raises_exception(self):
        with self.assertRaises(docopt.DocoptExit) as raised:
            main(['history'])

        self.assertIn('Usage:', str(raised.exception))

    def test_history_with_no_permissions(self):
        with self.assertRaises(SystemExit):
            main(['history', 'snap-test'])

        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?',
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_history_with_3rd_party_snap(self, mock_account_api):
        mock_account_api.return_value = {}

        with self.assertRaises(SystemExit):
            main(['history', 'snap-test'])

        self.assertIn(
            "Snap 'snap-test' was not found.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_history_with_3rd_party_snap_by_arch(self, mock_account_api):
        mock_account_api.return_value = {}

        with self.assertRaises(SystemExit):
            main(['history', 'snap-test', '--arch=arm64'])

        self.assertIn(
            "Snap 'snap-test' for 'arm64' was not found.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_history_with_3rd_party_snap_by_series(self, mock_account_api):
        mock_account_api.return_value = {}

        with self.assertRaises(SystemExit):
            main(['history', 'snap-test', '--series=15'])

        self.assertIn(
            "Snap 'snap-test' was not found in '15' series.",
            self.fake_logger.output)

    @mock.patch.object(storeapi.StoreClient, 'get_snap_history')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_history(self, mock_account_api, mock_history):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        mock_history.return_value = self.expected

        main(['history', 'snap-test'])

        mock_history.assert_called_once_with('snap-test', None, None)

        terminal_output = fake_terminal.getvalue()
        expected_output = """
  Rev.  Uploaded                 Arch    Version     Channels
------  -----------------------  ------  ----------  -------------
     2  2016-09-27T19:23:40.409  i386    2.0.1       -
     1  2016-09-27T18:38:43.388  amd64   2.0.1-test  stable*, edge
"""
        self.assertIn(expected_output.strip(), terminal_output)

    @mock.patch.object(storeapi.StoreClient, 'get_snap_history')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_history_by_arch(self, mock_account_api, mock_history):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        mock_history.return_value = self.expected

        main(['history', 'snap-test', '--arch=i386'])

        mock_history.assert_called_once_with('snap-test', None, 'i386')

        terminal_output = fake_terminal.getvalue()
        expected_output = """
  Rev.  Uploaded                 Arch    Version     Channels
------  -----------------------  ------  ----------  -------------
     2  2016-09-27T19:23:40.409  i386    2.0.1       -
"""
        self.assertIn(expected_output.strip(), terminal_output)

    @mock.patch.object(storeapi.StoreClient, 'get_snap_history')
    @mock.patch.object(storeapi.StoreClient, 'get_account_information')
    def test_history_by_series(self, mock_account_api, mock_history):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        mock_history.return_value = self.expected

        main(['history', 'snap-test', '--series=16'])

        mock_history.assert_called_once_with('snap-test', '16', None)

        terminal_output = fake_terminal.getvalue()
        expected_output = """
  Rev.  Uploaded                 Arch    Version     Channels
------  -----------------------  ------  ----------  -------------
     2  2016-09-27T19:23:40.409  i386    2.0.1       -
     1  2016-09-27T18:38:43.388  amd64   2.0.1-test  stable*, edge
"""
        self.assertIn(expected_output.strip(), terminal_output)
