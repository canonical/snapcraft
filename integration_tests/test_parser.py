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

import os
import fixtures
import subprocess

import integration_tests
import testscenarios
from snapcraft.tests import fixture_setup


class ParserTestCase(integration_tests.TestCase):
    """Test bin/snapcraft-parser"""

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable('TMPDIR', self.path))

    def call_parser(self, wiki_path, expect_valid, expect_output=True):
        part_file = os.path.join(self.path, 'parts.yaml')
        args = ['--index', wiki_path, '--output', part_file]

        if expect_valid:
            self.run_snapcraft_parser(args)
        else:
            self.assertRaises(
                subprocess.CalledProcessError,
                self.run_snapcraft_parser, args)

        self.assertEqual(os.path.exists(part_file), expect_output)


class TestParser(ParserTestCase):
    """Test bin/snapcraft-parser"""

    def test_parser_basic(self):
        """Test snapcraft-parser basic usage"""
        fixture = fixture_setup.FakePartsWiki()
        self.useFixture(fixture)

        self.call_parser(
            fixture.fake_parts_wiki_fixture.url, expect_valid=True)


class TestParserWikis(testscenarios.WithScenarios, ParserTestCase):
    """Test bin/snapcraft-parser"""

    scenarios = [
        ('Hidden .snapcraft.yaml file',
            {'wiki_file': 'hidden_parts_wiki',
             'expect_valid': True, 'expect_output': True}),
        ('Both snapcraft.yaml and .snapcraft.yaml files',
            {'wiki_file': 'both_parts_wiki',
             'expect_valid': False, 'expect_output': True}),
        ('Missing .snapcraft.yaml file',
            {'wiki_file': 'missing_parts_wiki',
             'expect_valid': False, 'expect_output': True}),
        ('Origin type, branch and commit options',
            {'wiki_file': 'origin_options_wiki',
             'expect_valid': True, 'expect_output': True}),
        ('Origin type, branch and commit options (wrong values)',
            {'wiki_file': 'wrong_origin_options_wiki',
             'expect_valid': False, 'expect_output': False}),
    ]

    def test_parse_wiki(self):
        wiki_file = os.path.join(os.path.dirname(__file__), self.wiki_file)
        self.call_parser(wiki_file, self.expect_valid, self.expect_output)
