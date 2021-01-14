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
import snapcraft.internal.project_loader.grammar._on as on
from snapcraft.internal.project_loader import grammar


def load_tests(loader, tests, ignore):
    tests.addTests(doctest.DocTestSuite(on))
    return tests


class TestOnStatementGrammar:

    scenarios = [
        (
            "on amd64",
            {
                "on_arch": "on amd64",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": ["foo"],
            },
        ),
        (
            "on i386",
            {
                "on_arch": "on amd64",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "i686",
                "expected_packages": list(),
            },
        ),
        (
            "ignored else",
            {
                "on_arch": "on amd64",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "host_arch": "x86_64",
                "expected_packages": ["foo"],
            },
        ),
        (
            "used else",
            {
                "on_arch": "on amd64",
                "body": ["foo"],
                "else_bodies": [["bar"]],
                "host_arch": "i686",
                "expected_packages": ["bar"],
            },
        ),
        (
            "third else ignored",
            {
                "on_arch": "on amd64",
                "body": ["foo"],
                "else_bodies": [["bar"], ["baz"]],
                "host_arch": "i686",
                "expected_packages": ["bar"],
            },
        ),
        (
            "third else followed",
            {
                "on_arch": "on amd64",
                "body": ["foo"],
                "else_bodies": [[{"on armhf": ["bar"]}], ["baz"]],
                "host_arch": "i686",
                "expected_packages": ["baz"],
            },
        ),
        (
            "nested amd64",
            {
                "on_arch": "on amd64",
                "body": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": ["foo"],
            },
        ),
        (
            "nested i386",
            {
                "on_arch": "on i386",
                "body": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}],
                "else_bodies": [],
                "host_arch": "i686",
                "expected_packages": ["bar"],
            },
        ),
        (
            "nested body ignored else",
            {
                "on_arch": "on amd64",
                "body": [{"on amd64": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": ["foo"],
            },
        ),
        (
            "nested body used else",
            {
                "on_arch": "on i386",
                "body": [{"on amd64": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "host_arch": "i686",
                "expected_packages": ["bar"],
            },
        ),
        (
            "nested else ignored else",
            {
                "on_arch": "on armhf",
                "body": ["foo"],
                "else_bodies": [[{"on amd64": ["bar"]}, {"else": ["baz"]}]],
                "host_arch": "x86_64",
                "expected_packages": ["bar"],
            },
        ),
        (
            "nested else used else",
            {
                "on_arch": "on armhf",
                "body": ["foo"],
                "else_bodies": [[{"on amd64": ["bar"]}, {"else": ["baz"]}]],
                "host_arch": "i686",
                "expected_packages": ["baz"],
            },
        ),
    ]

    def test(
        self, monkeypatch, on_arch, body, else_bodies, host_arch, expected_packages
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        processor = grammar.GrammarProcessor(
            None, snapcraft.ProjectOptions(), lambda x: True
        )
        statement = on.OnStatement(on=on_arch, body=body, processor=processor)

        for else_body in else_bodies:
            statement.add_else(else_body)

        assert statement.process() == expected_packages


class TestOnStatementInvalidGrammar:

    scenarios = [
        (
            "spaces in selectors",
            {
                "on_arch": "on amd64, ubuntu",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": ".*not a valid 'on' clause.*spaces are not allowed in the "
                "selectors.*",
            },
        ),
        (
            "beginning with comma",
            {
                "on_arch": "on ,amd64",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": ".*not a valid 'on' clause",
            },
        ),
        (
            "ending with comma",
            {
                "on_arch": "on amd64,",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": ".*not a valid 'on' clause",
            },
        ),
        (
            "multiple commas",
            {
                "on_arch": "on amd64,,ubuntu",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": ".*not a valid 'on' clause",
            },
        ),
        (
            "invalid selector format",
            {
                "on_arch": "on",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": ".*not a valid 'on' clause.*selectors are missing",
            },
        ),
        (
            "not even close",
            {
                "on_arch": "im-invalid",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": ".*not a valid 'on' clause",
            },
        ),
    ]

    def test(self, on_arch, body, else_bodies, expected_exception):
        with pytest.raises(grammar.errors.OnStatementSyntaxError) as error:
            processor = grammar.GrammarProcessor(
                None, snapcraft.ProjectOptions(), lambda x: "invalid" not in x
            )
            statement = on.OnStatement(on=on_arch, body=body, processor=processor)

            for else_body in else_bodies:
                statement.add_else(else_body)

            statement.process()

        assert re.match(expected_exception, str(error.value))


def test_else_fail(monkeypatch):
    monkeypatch.setattr(platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

    processor = grammar.GrammarProcessor(
        None, snapcraft.ProjectOptions(), lambda x: True
    )
    statement = on.OnStatement(on="on i386", body=["foo"], processor=processor)

    statement.add_else(None)

    with pytest.raises(grammar.errors.UnsatisfiedStatementError) as error:
        statement.process()

    assert str(error.value) == "Unable to satisfy 'on i386', failure forced"
