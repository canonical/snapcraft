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
import textwrap

import fixtures

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests,
)

account_info_data = {
    'snaps': {
        '16': {
            'ubuntu-core': {'snap-id': 'good'},
        }
    }
}


class GatedTestCase(tests.TestCase):

    command_name = 'gated'

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_store = tests.fixture_setup.FakeStore()
        self.useFixture(self.fake_store)
        self.client = storeapi.StoreClient()
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    def test_gated_unknown_snap(self):
        self.client.login('dummy', 'test correct password')

        self.assertRaises(
            SystemExit,
            main, [self.command_name, 'notfound'])
        self.assertIn("Snap 'notfound' was not found.",
                      self.fake_logger.output)

    def test_gated_success(self):
        self.client.login('dummy', 'test correct password')

        main([self.command_name, 'ubuntu-core'])

        expected_output = textwrap.dedent("""\
            Name      Revision  Required    Approved
            snap-1           3  True        2016-09-19T21:07:27Z
            snap-2           5  False       2016-09-19T21:07:27Z
            snap-3           -  True        2016-09-19T21:07:27Z""")
        self.assertIn(expected_output, self.fake_terminal.getvalue())

    def test_gated_no_validations(self):
        self.client.login('dummy', 'test correct password')

        main([self.command_name, 'basic'])

        expected_output = "There are no validations for snap 'basic'\n"
        self.assertEqual(expected_output, self.fake_terminal.getvalue())

    def test_no_login(self):
        self.assertRaises(
            SystemExit,
            main, [self.command_name, 'ubuntu-core'])
        self.assertIn('No valid credentials found. Have you run',
                      self.fake_logger.output)
