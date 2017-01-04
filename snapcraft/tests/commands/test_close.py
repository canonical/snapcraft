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


class ChannelClosingTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    def test_close_missing_permission(self, mock_get_account_info):
        mock_get_account_info.return_value = {
            'account_id': 'abcd',
            'snaps': {
            }
        }
        raised = self.assertRaises(
            SystemExit,
            main, ['close', 'foo', 'beta'])

        self.assertEqual(1, raised.code)
        self.assertEqual([
            'Your account lacks permission to close channels for this snap. '
            'Make sure the logged in account has upload permissions on '
            "'foo' in series '16'.",
        ], self.fake_logger.output.splitlines())

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.SCAClient, 'close_channels')
    def test_close_basic(self, mock_close_channels, mock_get_account_info):
        mock_get_account_info.return_value = {
            'snaps': {
                '16': {'basic': {'snap-id': 'snap-id'}}
            }
        }
        closed_channels = ['beta']
        channel_maps = {
            'amd64': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'channel': 'beta', 'info': 'specific',
                 'version': '1.1', 'revision': 42},
                {'channel': 'edge', 'info': 'tracking'}
            ],
        }
        mock_close_channels.side_effect = [
            (closed_channels, channel_maps),
        ]

        main(['close', 'basic', 'beta'])

        self.assertEqual([
            'Arch    Channel    Version    Revision',
            'amd64   stable     -          -',
            '        candidate  -          -',
            '        beta       1.1        42',
            '        edge       ^          ^',
            '',
            '\x1b[0;32mThe beta channel is now closed.\x1b[0m'
        ], self.fake_terminal.getvalue().splitlines())

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.SCAClient, 'close_channels')
    def test_close_multiple_channels(
            self, mock_close_channels, mock_get_account_info):
        mock_get_account_info.return_value = {
            'snaps': {
                '16': {'basic': {'snap-id': 'snap-id'}}
            }
        }
        closed_channels = ['beta', 'edge']
        channel_maps = {
            'amd64': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'specific',
                 'version': '1.1', 'revision': 42},
                {'channel': 'beta', 'info': 'tracking'},
                {'channel': 'edge', 'info': 'tracking'}
            ],
        }
        mock_close_channels.side_effect = [
            (closed_channels, channel_maps),
        ]

        main(['close', 'basic', 'beta', 'edge'])

        self.assertEqual([
            'Arch    Channel    Version    Revision',
            'amd64   stable     -          -',
            '        candidate  1.1        42',
            '        beta       ^          ^',
            '        edge       ^          ^',
            '',
            '\x1b[0;32mThe beta and edge channels are now closed.\x1b[0m'
        ], self.fake_terminal.getvalue().splitlines())

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.SCAClient, 'close_channels')
    def test_close_multiple_architectures(
            self, mock_close_channels, mock_get_account_info):
        mock_get_account_info.return_value = {
            'snaps': {
                '16': {'basic': {'snap-id': 'snap-id'}}
            }
        }
        closed_channels = ['beta']
        channel_maps = {
            'amd64': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'channel': 'beta', 'info': 'specific',
                 'version': '1.1', 'revision': 42},
                {'channel': 'edge', 'info': 'tracking'}
            ],
            'armhf': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'beta', 'info': 'specific',
                 'version': '1.2', 'revision': 24},
                {'channel': 'beta', 'info': 'tracking'},
                {'channel': 'edge', 'info': 'tracking'}
            ],
        }
        mock_close_channels.side_effect = [
            (closed_channels, channel_maps),
        ]

        main(['close', 'basic', 'beta'])

        self.assertEqual([
            'Arch    Channel    Version    Revision',
            'amd64   stable     -          -',
            '        candidate  -          -',
            '        beta       1.1        42',
            '        edge       ^          ^',
            'armhf   stable     -          -',
            '        beta       1.2        24',
            '        beta       ^          ^',
            '        edge       ^          ^',
            '',
            '\x1b[0;32mThe beta channel is now closed.\x1b[0m'
        ], self.fake_terminal.getvalue().splitlines())
