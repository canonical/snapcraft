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

import snapcraft
from snapcraft.internal.project_loader import grammar
import snapcraft.internal.project_loader.grammar._on as on
import snapcraft.internal.project_loader.grammar._to as to
import snapcraft.internal.project_loader.grammar._compound as compound

from . import GrammarBaseTestCase


class CompoundStatementGrammarTestCase(GrammarBaseTestCase):

    scenarios = [
        (
            "on amd64",
            {
                "on": "on amd64",
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "on i386",
            {
                "on": "on amd64",
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "i686",
                "expected_packages": set(),
            },
        ),
        (
            "ignored else",
            {
                "on": "on amd64",
                "to": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "host_arch": "x86_64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "used else",
            {
                "on": "on amd64",
                "to": "to i386",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "host_arch": "i686",
                "expected_packages": {"bar"},
            },
        ),
        (
            "third else ignored",
            {
                "on": "on amd64",
                "to": "to i386",
                "body": ["foo"],
                "else_bodies": [["bar"], ["baz"]],
                "host_arch": "i686",
                "expected_packages": {"bar"},
            },
        ),
        (
            "third else followed",
            {
                "on": "on amd64",
                "to": "to i386",
                "body": ["foo"],
                "else_bodies": [[{"on armhf": ["bar"]}], ["baz"]],
                "host_arch": "i686",
                "expected_packages": {"baz"},
            },
        ),
        (
            "nested amd64",
            {
                "on": "on amd64",
                "to": "to armhf",
                "body": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "nested i386",
            {
                "on": "on i386",
                "to": "to armhf",
                "body": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}],
                "else_bodies": [],
                "host_arch": "i686",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested body ignored else",
            {
                "on": "on amd64",
                "to": "to armhf",
                "body": [{"on amd64": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "nested body used else",
            {
                "on": "on i386",
                "to": "to armhf",
                "body": [{"on amd64": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "host_arch": "i686",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested else ignored else",
            {
                "on": "on armhf",
                "to": "to i386",
                "body": ["foo"],
                "else_bodies": [[{"on amd64": ["bar"]}, {"else": ["baz"]}]],
                "host_arch": "x86_64",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested else used else",
            {
                "on": "on armhf",
                "to": "to i386",
                "body": ["foo"],
                "else_bodies": [[{"on amd64": ["bar"]}, {"else": ["baz"]}]],
                "host_arch": "i686",
                "expected_packages": {"baz"},
            },
        ),
        (
            "with hyphen",
            {
                "on": "on other-arch",
                "to": "to yet-another-arch",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": set(),
            },
        ),
        (
            "multiple selectors",
            {
                "on": "on amd64,i386",
                "to": "to armhf,arm64",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": set(),
            },
        ),
    ]

    @patch("platform.architecture")
    @patch("platform.machine")
    def test_compound_statement_grammar(
        self, platform_machine_mock, platform_architecture_mock
    ):
        platform_machine_mock.return_value = self.host_arch
        platform_architecture_mock.return_value = ("64bit", "ELF")
        processor = grammar.GrammarProcessor(
            None, snapcraft.ProjectOptions(target_deb_arch="armhf"), self.checker
        )
        statements = [
            on.OnStatement(on=self.on, body=None, processor=processor),
            to.ToStatement(to=self.to, body=None, processor=processor),
        ]
        statement = compound.CompoundStatement(
            statements=statements, body=self.body, processor=processor
        )

        for else_body in self.else_bodies:
            statement.add_else(else_body)

        self.assertThat(statement.process(), Equals(self.expected_packages))


class CompoundStatementInvalidGrammarTestCase(GrammarBaseTestCase):

    scenarios = [
        (
            "spaces in on selectors",
            {
                "on": "on amd64, ubuntu",
                "to": "to i386",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": grammar.errors.OnStatementSyntaxError,
                "expected_message": ".*not a valid 'on' clause.*spaces are not allowed in the "
                "selectors.*",
            },
        ),
        (
            "spaces in to selectors",
            {
                "on": "on amd64,ubuntu",
                "to": "to i386, armhf",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": grammar.errors.ToStatementSyntaxError,
                "expected_message": ".*not a valid 'to' clause.*spaces are not allowed in the "
                "selectors.*",
            },
        ),
    ]

    def test_on_statement_invalid_grammar(self):
        with testtools.ExpectedException(
            self.expected_exception, self.expected_message
        ):
            processor = grammar.GrammarProcessor(
                None, snapcraft.ProjectOptions(target_deb_arch="armhf"), self.checker
            )
            statements = [
                on.OnStatement(on=self.on, body=None, processor=processor),
                to.ToStatement(to=self.to, body=None, processor=processor),
            ]
            statement = compound.CompoundStatement(
                statements=statements, body=self.body, processor=processor
            )

            for else_body in self.else_bodies:
                statement.add_else(else_body)

            statement.process()
