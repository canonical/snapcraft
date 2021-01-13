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

import platform
import re

import pytest

import snapcraft
import snapcraft.internal.project_loader.grammar._compound as compound
import snapcraft.internal.project_loader.grammar._on as on
import snapcraft.internal.project_loader.grammar._to as to
from snapcraft.internal.project_loader import grammar


class TestCompoundStatementGrammar:

    scenarios = [
        (
            "on amd64",
            {
                "on_arch": "on amd64",
                "to_arch": "to armhf",
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
                "to_arch": "to armhf",
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
                "to_arch": "to armhf",
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
                "to_arch": "to i386",
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
                "to_arch": "to i386",
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
                "to_arch": "to i386",
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
                "to_arch": "to armhf",
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
                "to_arch": "to armhf",
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
                "to_arch": "to armhf",
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
                "to_arch": "to armhf",
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
                "to_arch": "to i386",
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
                "to_arch": "to i386",
                "body": ["foo"],
                "else_bodies": [[{"on amd64": ["bar"]}, {"else": ["baz"]}]],
                "host_arch": "i686",
                "expected_packages": ["baz"],
            },
        ),
        (
            "with hyphen",
            {
                "on_arch": "on other-arch",
                "to_arch": "to yet-another-arch",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": list(),
            },
        ),
        (
            "multiple selectors",
            {
                "on_arch": "on amd64,i386",
                "to_arch": "to armhf,arm64",
                "body": ["foo"],
                "else_bodies": [],
                "host_arch": "x86_64",
                "expected_packages": list(),
            },
        ),
    ]

    def test(
        self,
        monkeypatch,
        on_arch,
        to_arch,
        body,
        else_bodies,
        host_arch,
        expected_packages,
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        processor = grammar.GrammarProcessor(
            None, snapcraft.ProjectOptions(target_deb_arch="armhf"), lambda x: True
        )
        statements = [
            on.OnStatement(on=on_arch, body=None, processor=processor),
            to.ToStatement(to=to_arch, body=None, processor=processor),
        ]
        statement = compound.CompoundStatement(
            statements=statements, body=body, processor=processor
        )

        for else_body in else_bodies:
            statement.add_else(else_body)

        assert statement.process() == expected_packages


class TestCompoundStatementInvalidGrammar:

    scenarios = [
        (
            "spaces in on selectors",
            {
                "on_arch": "on amd64, ubuntu",
                "to_arch": "to i386",
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
                "on_arch": "on amd64,ubuntu",
                "to_arch": "to i386, armhf",
                "body": ["foo"],
                "else_bodies": [],
                "expected_exception": grammar.errors.ToStatementSyntaxError,
                "expected_message": ".*not a valid 'to' clause.*spaces are not allowed in the "
                "selectors.*",
            },
        ),
    ]

    def test(
        self, on_arch, to_arch, body, else_bodies, expected_exception, expected_message
    ):
        with pytest.raises(expected_exception) as error:
            processor = grammar.GrammarProcessor(
                None,
                snapcraft.ProjectOptions(target_deb_arch="armhf"),
                lambda x: "invalid" not in x,
            )
            statements = [
                on.OnStatement(on=on_arch, body=None, processor=processor),
                to.ToStatement(to=to_arch, body=None, processor=processor),
            ]
            statement = compound.CompoundStatement(
                statements=statements, body=body, processor=processor
            )

            for else_body in else_bodies:
                statement.add_else(else_body)

            statement.process()

        assert re.match(expected_message, str(error.value))
