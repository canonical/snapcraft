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

from snapcraft import tests
from snapcraft.integrations import travis
from snapcraft.main import main


class EnableCITestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    def test_enable_ci_empty(self):
        raised = self.assertRaises(
            SystemExit,
            main, ['enable-ci'])

        self.assertEqual(1, raised.code)
        self.assertEqual([
            'Please select one of the supported integration systems: travis.'
        ], self.fake_logger.output.splitlines())

    def test_enable_ci_unknown(self):
        raised = self.assertRaises(
            SystemExit,
            main, ['enable-ci', 'bazinga'])

        self.assertEqual(1, raised.code)
        self.assertEqual([
            '"bazinga" integration is not supported by snapcraft.',
            'Please select one of the supported integration systems: travis.'
        ], self.fake_logger.output.splitlines())

    @mock.patch.object(travis, '__doc__')
    @mock.patch.object(travis, 'enable')
    @mock.patch('builtins.input')
    def test_enable_ci_travis(self, mock_input, mock_enable, mock_doc):
        mock_input.side_effect = ['y']
        mock_doc.__str__.return_value = '<module docstring>'

        main(['enable-ci', 'travis'])

        self.assertEqual(1, mock_enable.call_count)
        self.assertEqual(
            '<module docstring>\n', self.fake_terminal.getvalue())

    @mock.patch.object(travis, 'refresh')
    def test_enable_ci_travis_refresh(self, mock_refresh):

        main(['enable-ci', 'travis', '--refresh'])

        self.assertEqual(1, mock_refresh.call_count)
        self.assertEqual('', self.fake_terminal.getvalue())
