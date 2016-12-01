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

        tempdir = fixtures.TempDir()
        self.useFixture(tempdir)
        self.useFixture(fixtures.EnvironmentVariable('TMPDIR', tempdir.path))

    def call_parser(self, wiki_path, expect_valid):
        args = [self.snapcraft_parser_command, '--index', wiki_path,
                '--debug',
                '--output', 'parts.yaml']

        if expect_valid:
            subprocess.check_call(args, stderr=subprocess.DEVNULL,
                                  stdout=subprocess.DEVNULL)
        else:
            self.assertRaises(
                subprocess.CalledProcessError,
                subprocess.check_call, args, stderr=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL)

        self.assertTrue(os.path.exists('parts.yaml'))


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
            {'wiki_file': 'hidden_parts_wiki', 'expect_valid': True}),
        ('Both snapcraft.yaml and .snapcraft.yaml files',
            {'wiki_file': 'both_parts_wiki', 'expect_valid': False}),
        ('Missing .snapcraft.yaml file',
            {'wiki_file': 'missing_parts_wiki', 'expect_valid': False}),
        ('Origin type, branch and commit options',
            {'wiki_file': 'origin_options_wiki', 'expect_valid': True}),
    ]

    def test_parse_wiki(self):
        wiki_file = os.path.join(os.path.dirname(__file__), self.wiki_file)
        self.call_parser(wiki_file, self.expect_valid)
