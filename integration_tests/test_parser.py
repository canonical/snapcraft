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
import subprocess

import integration_tests
from snapcraft.tests import fixture_setup


class TestParser(integration_tests.TestCase):
    """Test bin/snapcraft-parser"""

    def test_parser_basic(self):
        """Test snapcraft-parser basic usage"""
        fixture = fixture_setup.FakePartsWiki()
        self.useFixture(fixture)

        args = ['--index',
                fixture.fake_parts_wiki_fixture.url,
                '--output', 'parts.yaml']
        self.run_snapcraft_parser(args)

        self.assertTrue(os.path.exists('parts.yaml'))

    def test_hidden_snapcraft_yaml(self):
        """Test hidden .snapcraft.yaml file."""
        args = ['--index',
                os.path.join(os.path.dirname(__file__), 'hidden_parts_wiki'),
                '--output', 'parts.yaml']
        self.run_snapcraft_parser(args)

        self.assertTrue(os.path.exists('parts.yaml'))

    def test_both_snapcraft_yaml(self):
        """Test hidden .snapcraft.yaml file."""
        args = ['--index',
                os.path.join(os.path.dirname(__file__), 'both_parts_wiki'),
                '--output', 'parts.yaml']
        self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft_parser, args)

        self.assertTrue(os.path.exists('parts.yaml'))

    def test_missing_snapcraft_yaml(self):
        """Test missing .snapcraft.yaml file."""
        args = ['--index',
                os.path.join(os.path.dirname(__file__), 'missing_parts_wiki'),
                '--output', 'parts.yaml']
        self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft_parser,  args)

        self.assertTrue(os.path.exists('parts.yaml'))
