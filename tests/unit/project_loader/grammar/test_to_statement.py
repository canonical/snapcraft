# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017, 2018 Canonical Ltd
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
from unittest.mock import patch

import snapcraft
from snapcraft.internal.project_loader import grammar
import snapcraft.internal.project_loader.grammar._to as to

from . import GrammarBaseTestCase


def load_tests(loader, tests, ignore):
    tests.addTests(doctest.DocTestSuite(to))
    return tests


class ToStatementGrammarTestCase(GrammarBaseTestCase):

    scenarios = [
        (
            "no target arch",
            {
                "to": "to amd64",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": None,
                "expected_packages": {"foo"},
            },
        ),
        (
            "amd64 to armhf",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": {"foo"},
            },
        ),
        (
            "amd64 to armhf, arch specified",
            {
                "to": "to armhf",
                "body": ["foo:amd64"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": {"foo:amd64"},
            },
        ),
        (
            "amd64 to i386",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "i386",
                "expected_packages": set(),
            },
        ),
        (
            "ignored else",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "target_arch": "armhf",
                "expected_packages": {"foo"},
            },
        ),
        (
            "used else",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "target_arch": "i386",
                "expected_packages": {"bar"},
            },
        ),
        (
            "used else, arch specified",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar:amd64"]],
                "target_arch": "i386",
                "expected_packages": {"bar:amd64"},
            },
        ),
        (
            "third else ignored",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar"], ["baz"]],
                "target_arch": "i386",
                "expected_packages": {"bar"},
            },
        ),
        (
            "third else followed",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [[{"to armhf": ["bar"]}], ["baz"]],
                "target_arch": "i386",
                "expected_packages": {"baz"},
            },
        ),
        (
            "nested armhf",
            {
                "to": "to armhf",
                "body": [{"to armhf": ["foo"]}, {"to i386": ["bar"]}],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": {"foo"},
            },
        ),
        (
            "nested i386",
            {
                "to": "to i386",
                "body": [{"to armhf": ["foo"]}, {"to i386": ["bar"]}],
                "else_bodies": [],
                "target_arch": "i386",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested body ignored else",
            {
                "to": "to armhf",
                "body": [{"to armhf": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": {"foo"},
            },
        ),
        (
            "nested body used else",
            {
                "to": "to i386",
                "body": [{"to armhf": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "target_arch": "i386",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested else ignored else",
            {
                "to": "to i386",
                "body": ["foo"],
                "else_bodies": [[{"to armhf": ["bar"]}, {"else": ["baz"]}]],
                "target_arch": "armhf",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested else used else",
            {
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [[{"to armhf": ["bar"]}, {"else": ["baz"]}]],
                "target_arch": "i386",
                "expected_packages": {"baz"},
            },
        ),
    ]

    @patch("platform.architecture")
    @patch("platform.machine")
    def test_to_statement_grammar(
        self, platform_machine_mock, platform_architecture_mock
    ):
        platform_machine_mock.return_value = "x86_64"
        platform_architecture_mock.return_value = ("64bit", "ELF")
        processor = grammar.GrammarProcessor(
            None,
            snapcraft.ProjectOptions(target_deb_arch=self.target_arch),
            self.checker,
        )
        statement = to.ToStatement(to=self.to, body=self.body, processor=processor)

        for else_body in self.else_bodies:
            statement.add_else(else_body)

        self.assertThat(statement.process(), Equals(self.expected_packages))


class ToStatementInvalidGrammarTestCase(GrammarBaseTestCase):

    scenarios = [
        (
            "spaces in selectors",
            {
                "to": "to armhf, ubuntu",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause.*spaces are not allowed in the "
                "selectors.*",
            },
        ),
        (
            "beginning with comma",
            {
                "to": "to ,armhf",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
        (
            "ending with comma",
            {
                "to": "to armhf,",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
        (
            "multiple commas",
            {
                "to": "to armhf,,ubuntu",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
        (
            "invalid selector format",
            {
                "to": "to",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause.*selectors are missing",
            },
        ),
        (
            "not even close",
            {
                "to": "im-invalid",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
    ]

    def test_to_statement_invalid_grammar(self):
        with testtools.ExpectedException(
            grammar.errors.ToStatementSyntaxError, self.expected_exception
        ):
            processor = grammar.GrammarProcessor(
                None,
                snapcraft.ProjectOptions(target_deb_arch=self.target_arch),
                self.checker,
            )
            statement = to.ToStatement(to=self.to, body=self.body, processor=processor)

            for else_body in self.else_bodies:
                statement.add_else(else_body)

            statement.process()


class ToStatementElseFail(GrammarBaseTestCase):
    @patch("platform.architecture")
    @patch("platform.machine")
    def test_else_fail(self, platform_machine_mock, platform_architecture_mock):
        platform_machine_mock.return_value = "x86_64"
        platform_architecture_mock.return_value = ("64bit", "ELF")
        processor = grammar.GrammarProcessor(
            None, snapcraft.ProjectOptions(target_deb_arch="i386"), self.checker
        )
        statement = to.ToStatement(to="to armhf", body=["foo"], processor=processor)

        statement.add_else(None)

        with testtools.ExpectedException(
            grammar.errors.UnsatisfiedStatementError,
            "Unable to satisfy 'to armhf', failure forced",
        ):
            statement.process()
