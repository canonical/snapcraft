# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import snapcraft
from snapcraft.tests import unit
from testtools.matchers import Equals

from snapcraft.internal.project_loader.grammar_processing import (
    PartGrammarProcessor,
    _part_grammar_processor as processor
)

import doctest
from unittest import mock


def load_tests(loader, tests, ignore):
    tests.addTests(doctest.DocTestSuite(processor))
    return tests


class PartGrammarTestCase(unit.TestCase):

    scenarios = [
        ('plain string', {
            'properties': {'plugin': 'dump',
                           'source': 'foo'},
            'target_arch': 'amd64',
            'expected': {'plugin': 'dump', 'source': 'foo'}
        }),
        ('on amd64', {
            'properties': {'plugin': 'dump',
                           'source': [{'on amd64': 'foo'}]},
            'target_arch': 'amd64',
            'expected': {'plugin': 'dump', 'source': 'foo'}
        }),
        ('on i386', {
            'properties': {'plugin': 'dump',
                           'source': [{'on i386': 'foo'}, {'else': 'bar'}]},
            'target_arch': 'amd64',
            'expected': {'plugin': 'dump', 'source': 'bar'}
        }),
        ('on i386, target_arch=i386', {
            'properties': {'plugin': 'dump',
                           'source': [{'on i386': 'foo'}, {'else': 'bar'}]},
            'target_arch': 'i386',
            'expected': {'plugin': 'dump', 'source': 'foo'}
        }),
    ]

    def test_string_grammar(self):
        repo = mock.Mock()
        repo.is_valid.return_value = True
        plugin = mock.Mock()
        plugin.properties = self.properties
        self.assertThat(PartGrammarProcessor(
            plugin=plugin,
            properties=plugin.properties,
            project_options=snapcraft.ProjectOptions(
                target_deb_arch=self.target_arch),
            repo=repo).get_properties(),
                        Equals(self.expected))
