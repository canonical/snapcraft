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


class ListSnapsTestCase(tests.TestCase):

    scenarios = [
        ('list-snaps', {'command_name': 'list-snaps'}),
        ('snaps alias', {'command_name': 'snaps'}),
    ]

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    def test_list_snaps_without_login(self):
        with self.assertRaises(SystemExit) as raised:
            main([self.command_name])

        self.assertEqual(1, raised.exception.code)
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    def test_list_snaps_successfully(self, mock_get_account_information):
        mock_get_account_information.return_value = {
            'snaps': {
                '16': {
                    'foo': {
                        'status': 'Approved',
                        'snap-id': 'a_snap_id',
                        'private': False
                    },
                    'bar': {
                        'status': 'ReviewPending',
                        'snap-id': 'another_snap_id',
                        'private': True
                    },
                },
            },
        }

        main([self.command_name])

        expected_output = dedent('''\
        Name    Status         Snap-Id          Private
        bar     ReviewPending  another_snap_id  True
        foo     Approved       a_snap_id        False
        ''')
        self.assertIn(expected_output, self.fake_terminal.getvalue())
