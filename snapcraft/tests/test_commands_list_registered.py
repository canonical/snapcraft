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


class ListRegisteredTestCase(tests.TestCase):

    scenarios = [
        ('list-registered', {'command_name': 'list-registered'}),
        ('registered alias', {'command_name': 'registered'}),
    ]

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    def test_list_registered_without_login(self):
        raised = self.assertRaises(
            SystemExit,
            main, [self.command_name])

        self.assertEqual(1, raised.code)
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    def test_list_registered_empty(self, mock_get_account_information):
        mock_get_account_information.return_value = {
            'snaps': {},
        }

        main([self.command_name])

        self.assertIn(
            "There are no registered snaps for series '16'.",
            self.fake_terminal.getvalue())

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    def test_list_registered_successfully(self, mock_get_account_information):
        mock_get_account_information.return_value = {
            'snaps': {
                '16': {
                    'foo': {
                        'status': 'Approved',
                        'snap-id': 'a_snap_id',
                        'private': False,
                        'since': '2016-12-12T01:01:01Z',
                        'price': '9.99',
                    },
                    'bar': {
                        'status': 'ReviewPending',
                        'snap-id': 'another_snap_id',
                        'private': True,
                        'since': '2016-12-12T01:01:01Z',
                        'price': None,
                    },
                    'baz': {
                        'status': 'Approved',
                        'snap-id': 'yet_another_snap_id',
                        'private': True,
                        'since': '2016-12-12T02:02:02Z',
                        'price': '6.66',
                    },
                    'boing': {
                        'status': 'Approved',
                        'snap-id': 'boing_snap_id',
                        'private': False,
                        'since': '2016-12-12T03:03:03Z',
                        'price': None,
                    },
                },
            },
        }

        main([self.command_name])

        expected_output = dedent('''\
        Name    Since                 Visibility    Price    Notes
        baz     2016-12-12T02:02:02Z  private       6.66     -
        boing   2016-12-12T03:03:03Z  public        -        -
        foo     2016-12-12T01:01:01Z  public        9.99     -
        ''')
        self.assertIn(expected_output, self.fake_terminal.getvalue())
