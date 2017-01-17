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


class ReleaseCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

    def test_upload_without_snap_must_raise_exception(self):
        raised = self.assertRaises(
            docopt.DocoptExit,
            main, ['release'])

        self.assertIn('Usage:', str(raised))

    def test_release_snap(self):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        patcher = mock.patch.object(storeapi.StoreClient, 'release')
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            'opened_channels': ['beta'],
            'channel_map': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'revision': 19, 'channel': 'beta', 'version': '0',
                 'info': 'specific'},
                {'channel': 'edge', 'info': 'tracking'}
            ]
        }

        main(['release', 'nil-snap', '19', 'beta'])

        mock_release.assert_called_once_with('nil-snap', '19', ['beta'])

        self.assertEqual([
            "\x1b[0;32mThe 'beta' channel is now open.\x1b[0m",
            '',
            'Channel    Version    Revision',
            'stable     -          -',
            'candidate  -          -',
            'beta       0          19',
            'edge       ^          ^',
        ], fake_terminal.getvalue().splitlines())

    def test_release_snap_opens_more_than_one_channel(self):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        patcher = mock.patch.object(storeapi.StoreClient, 'release')
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            'opened_channels': ['stable', 'beta', 'edge'],
            'channel_map': [
                {'channel': 'stable', 'info': 'none'},
                {'channel': 'candidate', 'info': 'none'},
                {'revision': 19, 'channel': 'beta', 'version': '0',
                 'info': 'specific'},
                {'channel': 'edge', 'info': 'tracking'}
            ]
        }

        main(['release', 'nil-snap', '19', 'beta'])

        mock_release.assert_called_once_with('nil-snap', '19', ['beta'])

        self.assertEqual([
            "\x1b[0;32mThe 'stable', 'beta' and 'edge' channels "
            "are now open.\x1b[0m",
            '',
            'Channel    Version    Revision',
            'stable     -          -',
            'candidate  -          -',
            'beta       0          19',
            'edge       ^          ^',
        ], fake_terminal.getvalue().splitlines())

    def test_release_with_bad_channel_info(self):
        patcher = mock.patch.object(storeapi.StoreClient, 'release')
        mock_release = patcher.start()
        self.addCleanup(patcher.stop)
        mock_release.return_value = {
            'channel_map': [
                {'channel': 'stable', 'info': 'fake-bad-channel-info'},
                {'channel': 'candidate', 'info': 'none'},
                {'revision': 19, 'channel': 'beta', 'version': '0',
                 'info': 'specific'},
                {'channel': 'edge', 'info': 'tracking'}
            ]
        }

        self.assertRaises(
            SystemExit,
            main, ['release', 'nil-snap', '19', 'beta'])

        self.assertIn(
            'Unexpected channel info \'fake-bad-channel-info\'.',
            self.fake_logger.output)

    def test_release_without_login_must_raise_exception(self):
        self.assertRaises(
            SystemExit,
            main, ['release', 'nil-snap', '19', 'beta'])
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)
