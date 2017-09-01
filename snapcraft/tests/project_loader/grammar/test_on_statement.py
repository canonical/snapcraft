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

import doctest
import testtools
from testtools.matchers import Equals

import snapcraft
from snapcraft.internal.project_loader import grammar
import snapcraft.internal.project_loader.grammar._on as on

from . import GrammarTestCase


def load_tests(loader, tests, ignore):
    tests.addTests(doctest.DocTestSuite(on))
    return tests


class OnStatementGrammarTestCase(GrammarTestCase):

    scenarios = [
        ('on amd64', {
            'on': 'on amd64',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_packages': {'foo'}
        }),
        ('on i386', {
            'on': 'on amd64',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'i386',
            'expected_packages': set()
        }),
        ('ignored else', {
            'on': 'on amd64',
            'body': ['foo'],
            'else_bodies': [
                ['bar']
            ],
            'target_arch': 'amd64',
            'expected_packages': {'foo'}
        }),
        ('used else', {
            'on': 'on amd64',
            'body': ['foo'],
            'else_bodies': [
                ['bar']
            ],
            'target_arch': 'i386',
            'expected_packages': {'bar'}
        }),
        ('third else ignored', {
            'on': 'on amd64',
            'body': ['foo'],
            'else_bodies': [
                ['bar'],
                ['baz']
            ],
            'target_arch': 'i386',
            'expected_packages': {'bar'}
        }),
        ('third else followed', {
            'on': 'on amd64',
            'body': ['foo'],
            'else_bodies': [
                [{'on armhf': ['bar']}],
                ['baz']
            ],
            'target_arch': 'i386',
            'expected_packages': {'baz'}
        }),
        ('nested amd64', {
            'on': 'on amd64',
            'body': [
                {'on amd64': ['foo']},
                {'on i386': ['bar']},
            ],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_packages': {'foo'}
        }),
        ('nested i386', {
            'on': 'on i386',
            'body': [
                {'on amd64': ['foo']},
                {'on i386': ['bar']},
            ],
            'else_bodies': [],
            'target_arch': 'i386',
            'expected_packages': {'bar'}
        }),
        ('nested body ignored else', {
            'on': 'on amd64',
            'body': [
                {'on amd64': ['foo']},
                {'else': ['bar']},
            ],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_packages': {'foo'}
        }),
        ('nested body used else', {
            'on': 'on i386',
            'body': [
                {'on amd64': ['foo']},
                {'else': ['bar']},
            ],
            'else_bodies': [],
            'target_arch': 'i386',
            'expected_packages': {'bar'}
        }),
        ('nested else ignored else', {
            'on': 'on armhf',
            'body': ['foo'],
            'else_bodies': [
                [
                    {'on amd64': ['bar']},
                    {'else': ['baz']},
                ],
            ],
            'target_arch': 'amd64',
            'expected_packages': {'bar'}
        }),
        ('nested else used else', {
            'on': 'on armhf',
            'body': ['foo'],
            'else_bodies': [
                [
                    {'on amd64': ['bar']},
                    {'else': ['baz']},
                ],
            ],
            'target_arch': 'i386',
            'expected_packages': {'baz'}
        }),
    ]

    def test_on_statement_grammar(self):
        options = snapcraft.ProjectOptions(target_deb_arch=self.target_arch)
        statement = on.OnStatement(
            on=self.on, body=self.body, project_options=options,
            checker=self.checker)

        for else_body in self.else_bodies:
            statement.add_else(else_body)

        self.assertThat(statement.process(), Equals(self.expected_packages))


class OnStatementInvalidGrammarTestCase(GrammarTestCase):

    scenarios = [
        ('spaces in selectors', {
            'on': 'on amd64, ubuntu',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_exception':
                ".*not a valid 'on' clause.*spaces are not allowed in the "
                'selectors.*',
        }),
        ('beginning with comma', {
            'on': 'on ,amd64',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_exception': ".*not a valid 'on' clause",
        }),
        ('ending with comma', {
            'on': 'on amd64,',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_exception': ".*not a valid 'on' clause",
        }),
        ('multiple commas', {
            'on': 'on amd64,,ubuntu',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_exception': ".*not a valid 'on' clause",
        }),
        ('invalid selector format', {
            'on': 'on',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_exception':
                ".*not a valid 'on' clause.*selectors are missing",
        }),
        ('not even close', {
            'on': 'im-invalid',
            'body': ['foo'],
            'else_bodies': [],
            'target_arch': 'amd64',
            'expected_exception': ".*not a valid 'on' clause",
        }),
    ]

    def test_on_statement_invalid_grammar(self):
        with testtools.ExpectedException(
                grammar.errors.OnStatementSyntaxError,
                self.expected_exception):
            options = snapcraft.ProjectOptions(
                target_deb_arch=self.target_arch)
            statement = on.OnStatement(
                on=self.on, body=self.body, project_options=options,
                checker=self.checker)

            for else_body in self.else_bodies:
                statement.add_else(else_body)

            statement.process()


class OnStatementElseFail(GrammarTestCase):

    def test_else_fail(self):
        options = snapcraft.ProjectOptions(
            target_deb_arch='amd64')
        statement = on.OnStatement(
            on='on i386', body=['foo'], project_options=options,
            checker=self.checker)

        statement.add_else(None)

        with testtools.ExpectedException(
                grammar.errors.UnsatisfiedStatementError,
                "Unable to satisfy 'on i386', failure forced"):
            statement.process()
