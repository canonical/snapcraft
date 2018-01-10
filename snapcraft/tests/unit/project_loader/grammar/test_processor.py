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

import testtools
from testtools.matchers import Equals
from unittest.mock import patch
from testscenarios.scenarios import multiply_scenarios

import snapcraft
from snapcraft.internal.project_loader import grammar

from . import GrammarTestCase


class GrammarOnDuplicatesTestCase(GrammarTestCase):

    scenarios = [
        ('same order', {
            'grammar': [
                {'on amd64,i386': ['foo']},
                {'on amd64,i386': ['bar']},
            ]}),
        ('different order', {
            'grammar': [
                {'on amd64,i386': ['foo']},
                {'on i386,amd64': ['bar']},
            ]}),
    ]

    def test_on_duplicates_raises(self):
        """Test that multiple identical selector sets is an error."""

        with testtools.ExpectedException(
                grammar.errors.GrammarSyntaxError,
                "Invalid grammar syntax: found duplicate 'on amd64,i386' "
                'statements. These should be merged.'):
            grammar.process_grammar(
                self.grammar, snapcraft.ProjectOptions(), self.checker)


class BasicGrammarTestCase(GrammarTestCase):

    scenarios = multiply_scenarios([
        ('unconditional', {
            'grammar': [
                'foo',
                'bar',
            ],
            'expected_packages': {'foo', 'bar'}
        }),
        ('mixed including', {
            'grammar': [
                'foo',
                {'on amd64': ['bar']}
            ],
            'expected_packages': {'foo', 'bar'}
        }),
        ('mixed excluding', {
            'grammar': [
                'foo',
                {'on i386': ['bar']}
            ],
            'expected_packages': {'foo'}
        }),
        ('on amd64', {
            'grammar': [
                {'on amd64': ['foo']},
                {'on i386': ['bar']},
            ],
            'expected_packages': {'foo'}
        }),
        ('on amd64 to i386', {
            'grammar': [
                {'on i386': ['foo']},
                {'on amd64': ['bar']},
            ],
            'expected_packages': {'bar'}
        }),
        ('ignored else', {
            'grammar': [
                {'on amd64': ['foo']},
                {'else': ['bar']},
            ],
            'expected_packages': {'foo'}
        }),
        ('used else', {
            'grammar': [
                {'on i386': ['foo']},
                {'else': ['bar']},
            ],
            'expected_packages': {'bar'}
        }),
        ('nested amd64', {
            'grammar': [
                {'on amd64': [
                    {'on amd64': ['foo']},
                    {'on i386': ['bar']},
                ]},
            ],
            'expected_packages': {'foo'}
        }),
        ('nested amd64, target i386', {
            'grammar': [
                {'on amd64': [
                    {'on i386': ['foo']},
                    {'on amd64': ['bar']},
                ]},
            ],
            'expected_packages': {'bar'}
        }),
        ('nested ignored else', {
            'grammar': [
                {'on amd64': [
                    {'on amd64': ['foo']},
                    {'else': ['bar']},
                ]},
            ],
            'expected_packages': {'foo'}
        }),
        ('nested used else', {
            'grammar': [
                {'on amd64': [
                    {'on i386': ['foo']},
                    {'else': ['bar']},
                ]},
            ],
            'expected_packages': {'bar'}
        }),
        ('try', {
            'grammar': [
                {'try': ['valid']},
            ],
            'expected_packages': {'valid'}
        }),
        ('try else', {
            'grammar': [
                {'try': ['invalid']},
                {'else': ['valid']},
            ],
            'expected_packages': {'valid'}
        }),
        ('nested try', {
            'grammar': [
                {'on amd64': [
                    {'try': ['foo']},
                    {'else': ['bar']},
                ]},
            ],
            'expected_packages': {'foo'}
        }),
        ('nested try else', {
            'grammar': [
                {'on amd64': [
                    {'try': ['invalid']},
                    {'else': ['bar']},
                ]},
            ],
            'expected_packages': {'bar'}
        }),
        ('optional', {
            'grammar': [
                'foo',
                {'try': ['invalid']},
            ],
            'expected_packages': {'foo'}
        }),
    ], [
        ('targeting amd64', {'target_arch': 'x86_64'}),
        ('targeting i386', {'target_arch': 'i686'})])

    @patch('platform.architecture')
    @patch('platform.machine')
    def test_basic_grammar(self, platform_machine_mock,
                           platform_architecture_mock):
        platform_machine_mock.return_value = 'x86_64'
        platform_architecture_mock.return_value = ('64bit', 'ELF')

        options = snapcraft.ProjectOptions(target_deb_arch=self.target_arch)
        self.assertThat(
            grammar.process_grammar(self.grammar, options, self.checker),
            Equals(self.expected_packages))


class InvalidGrammarTestCase(GrammarTestCase):

    scenarios = multiply_scenarios([
        ('unmatched else', {
            'grammar': [
                {'else': ['foo']}
            ],
            'expected_exception': ".*'else' doesn't seem to correspond.*",
        }),
        ('unmatched else fail', {
            'grammar': [
                'else fail'
            ],
            'expected_exception': ".*'else' doesn't seem to correspond.*",
        }),
        ('unexpected type', {
            'grammar': [
                5,
            ],
            'expected_exception': ".*expected grammar section.*but got.*",
        }),
    ], [
        ('targeting amd64', {'target_arch': 'x86_64'}),
        ('targeting i386', {'target_arch': 'i686'})])

    def test_invalid_grammar(self):
        with testtools.ExpectedException(
                grammar.errors.GrammarSyntaxError,
                self.expected_exception):
            grammar.process_grammar(
                self.grammar, snapcraft.ProjectOptions(), self.checker)
