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

import io
import logging
from unittest import mock

import fixtures

from snapcraft import main, tests
from snapcraft.internal import parts
from snapcraft.tests import fixture_setup


class SearchCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.FakeParts())
        with mock.patch('snapcraft.internal.parts.ProgressBar',
                        new=tests.SilentProgressBar):
            parts.update()

        patcher = mock.patch('snapcraft.internal.parts.get_terminal_width')
        self.mock_terminal = patcher.start()
        self.mock_terminal.return_value = 80
        self.addCleanup(patcher.stop)

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_searching_for_a_part_that_exists(self, mock_stdout):
        main.main(['search', 'curl'])

        expected_output = """PART NAME  DESCRIPTION
curl       test entry for curl
"""
        self.assertEqual(mock_stdout.getvalue(), expected_output)

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_empty_search_searches_all(self, mock_stdout):
        main.main(['search'])

        output = mock_stdout.getvalue()
        self.assertEqual(
            output.split('\n')[0], 'PART NAME            DESCRIPTION')
        self.assertTrue('part1                test entry for part1' in output)
        self.assertTrue('curl                 test entry for curl' in output)
        self.assertTrue(
            'long-described-part  this is a repetetive description '
            'this is a repetetive de...' in output)

    def test_searching_for_a_part_that_doesnt_exist_helps_out(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        main.main(['search', 'part that does not exist'])

        self.assertEqual(
            fake_logger.output,
            'No matches found, try to run `snapcraft update` to refresh the '
            'remote parts cache.\n')
