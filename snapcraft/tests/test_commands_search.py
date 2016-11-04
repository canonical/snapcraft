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

import fixtures

from snapcraft import main, tests
from snapcraft.tests import fixture_setup


class SearchCommandTestCase(tests.TestWithFakeRemoteParts):

    def test_searching_for_a_part_that_exists(self):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        main.main(['search', 'curl'])

        expected_output = """PART NAME  DESCRIPTION
curl       test entry for curl
"""
        self.assertEqual(fake_terminal.getvalue(), expected_output)

    def test_empty_search_searches_all(self):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        main.main(['search'])

        output = fake_terminal.getvalue()
        self.assertEqual(
            output.split('\n')[0], 'PART NAME            DESCRIPTION')
        self.assertTrue('part1                test entry for part1' in output)
        self.assertTrue('curl                 test entry for curl' in output)
        self.assertTrue(
            'long-described-part  this is a repetitive description '
            'this is a repetitive de...' in output)

    def test_search_trims_long_descriptions(self):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        main.main(['search', 'long-described-part'])

        expected_output = (
            'PART NAME            DESCRIPTION\n'
            'long-described-part  this is a repetitive description this is a '
            'repetitive de...\n')
        self.assertEqual(fake_terminal.getvalue(), expected_output)

    def test_search_only_first_line_of_description(self):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        main.main(['search', 'mulitline-part'])

        expected_output = (
            'PART NAME       DESCRIPTION\n'
            'multiline-part  this is a multiline description\n')
        self.assertEqual(fake_terminal.getvalue(), expected_output)

    def test_searching_for_a_part_that_doesnt_exist_helps_out(self):
        self.useFixture(fixture_setup.FakeTerminal())

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        main.main(['search', 'part that does not exist'])

        self.assertEqual(
            fake_logger.output,
            'No matches found, try to run `snapcraft update` to refresh the '
            'remote parts cache.\n')

    def test_search_on_non_tty(self):
        fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(fake_terminal)

        main.main(['search', 'curl'])

        expected_output = """PART NAME  DESCRIPTION
curl       test entry for curl
"""
        self.assertEqual(fake_terminal.getvalue(), expected_output)

    def test_search_output_alphabetical_order(self):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        main.main(['search'])
        output = fake_terminal.getvalue()
        expected_output = (
            'PART NAME            DESCRIPTION\n'
            'curl                 test entry for curl\n'
            'long-described-part  this is a repetitive description this is a '
            'repetitive de...\n'
            'multiline-part       this is a multiline description\n'
            'part1                test entry for part1\n')

        self.assertEqual(output, expected_output)
