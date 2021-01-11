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
import platform
import re

import pytest

import snapcraft
import snapcraft.internal.project_loader.grammar._to as to
from snapcraft.internal.project_loader import grammar


def load_tests(loader, tests, ignore):
    tests.addTests(doctest.DocTestSuite(to))
    return tests


class TestToStatementGrammar:

    scenarios = [
        (
            "no target arch",
            {
                "to_arch": "to amd64",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": None,
                "expected_packages": ["foo"],
            },
        ),
        (
            "amd64 to armhf",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": ["foo"],
            },
        ),
        (
            "amd64 to armhf, arch specified",
            {
                "to_arch": "to armhf",
                "body": ["foo:amd64"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": ["foo:amd64"],
            },
        ),
        (
            "amd64 to i386",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "i386",
                "expected_packages": list(),
            },
        ),
        (
            "ignored else",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "target_arch": "armhf",
                "expected_packages": ["foo"],
            },
        ),
        (
            "used else",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "target_arch": "i386",
                "expected_packages": ["bar"],
            },
        ),
        (
            "used else, arch specified",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar:amd64"]],
                "target_arch": "i386",
                "expected_packages": ["bar:amd64"],
            },
        ),
        (
            "third else ignored",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [["bar"], ["baz"]],
                "target_arch": "i386",
                "expected_packages": ["bar"],
            },
        ),
        (
            "third else followed",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [[{"to armhf": ["bar"]}], ["baz"]],
                "target_arch": "i386",
                "expected_packages": ["baz"],
            },
        ),
        (
            "nested armhf",
            {
                "to_arch": "to armhf",
                "body": [{"to armhf": ["foo"]}, {"to i386": ["bar"]}],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": ["foo"],
            },
        ),
        (
            "nested i386",
            {
                "to_arch": "to i386",
                "body": [{"to armhf": ["foo"]}, {"to i386": ["bar"]}],
                "else_bodies": [],
                "target_arch": "i386",
                "expected_packages": ["bar"],
            },
        ),
        (
            "nested body ignored else",
            {
                "to_arch": "to armhf",
                "body": [{"to armhf": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_packages": ["foo"],
            },
        ),
        (
            "nested body used else",
            {
                "to_arch": "to i386",
                "body": [{"to armhf": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "target_arch": "i386",
                "expected_packages": ["bar"],
            },
        ),
        (
            "nested else ignored else",
            {
                "to_arch": "to i386",
                "body": ["foo"],
                "else_bodies": [[{"to armhf": ["bar"]}, {"else": ["baz"]}]],
                "target_arch": "armhf",
                "expected_packages": ["bar"],
            },
        ),
        (
            "nested else used else",
            {
                "to_arch": "to armhf",
                "body": ["foo"],
                "else_bodies": [[{"to armhf": ["bar"]}, {"else": ["baz"]}]],
                "target_arch": "i386",
                "expected_packages": ["baz"],
            },
        ),
    ]

    def test(
        self, monkeypatch, to_arch, body, else_bodies, target_arch, expected_packages
    ):
        monkeypatch.setattr(platform, "machine", lambda: "x86_64")
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))
        processor = grammar.GrammarProcessor(
            None, snapcraft.ProjectOptions(target_deb_arch=target_arch), lambda x: True
        )
        statement = to.ToStatement(to=to_arch, body=body, processor=processor)

        for else_body in else_bodies:
            statement.add_else(else_body)

        assert statement.process() == expected_packages


class TestToStatementInvalidGrammar:

    scenarios = [
        (
            "spaces in selectors",
            {
                "to_arch": "to armhf, ubuntu",
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
                "to_arch": "to ,armhf",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
        (
            "ending with comma",
            {
                "to_arch": "to armhf,",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
        (
            "multiple commas",
            {
                "to_arch": "to armhf,,ubuntu",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
        (
            "invalid selector format",
            {
                "to_arch": "to_arch",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause.*selectors are missing",
            },
        ),
        (
            "not even close",
            {
                "to_arch": "im-invalid",
                "body": ["foo"],
                "else_bodies": [],
                "target_arch": "armhf",
                "expected_exception": ".*not a valid 'to' clause",
            },
        ),
    ]

    def test(self, to_arch, body, else_bodies, target_arch, expected_exception):
        with pytest.raises(grammar.errors.ToStatementSyntaxError) as error:
            processor = grammar.GrammarProcessor(
                None,
                snapcraft.ProjectOptions(target_deb_arch=target_arch),
                lambda x: True,
            )
            statement = to.ToStatement(to=to_arch, body=body, processor=processor)

            for else_body in else_bodies:
                statement.add_else(else_body)

            statement.process()

        assert re.match(expected_exception, str(error.value))


def test_else_fail(monkeypatch):
    monkeypatch.setattr(platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

    processor = grammar.GrammarProcessor(
        None, snapcraft.ProjectOptions(target_deb_arch="i386"), lambda x: True
    )
    statement = to.ToStatement(to="to armhf", body=["foo"], processor=processor)

    statement.add_else(None)

    with pytest.raises(grammar.errors.UnsatisfiedStatementError) as error:
        statement.process()

    assert str(error.value) == "Unable to satisfy 'to armhf', failure forced"
