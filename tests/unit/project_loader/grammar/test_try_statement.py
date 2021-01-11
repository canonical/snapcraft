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

import pytest

import snapcraft
import snapcraft.internal.project_loader.grammar._try as _try
from snapcraft.internal.project_loader import grammar


def load_tests(loader, tests, ignore):
    tests.addTests(doctest.DocTestSuite(_try))
    return tests


class TestTryStatementGrammar:

    scenarios = [
        (
            "followed body",
            {
                "body": ["foo", "bar"],
                "else_bodies": [],
                "expected_packages": ["foo", "bar"],
            },
        ),
        (
            "followed else",
            {
                "body": ["invalid"],
                "else_bodies": [["valid"]],
                "expected_packages": ["valid"],
            },
        ),
        (
            "optional without else",
            {"body": ["invalid"], "else_bodies": [], "expected_packages": list()},
        ),
        (
            "followed chained else",
            {
                "body": ["invalid1"],
                "else_bodies": [["invalid2"], ["finally-valid"]],
                "expected_packages": ["finally-valid"],
            },
        ),
        (
            "nested body followed body",
            {
                "body": [{"try": ["foo"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "expected_packages": ["foo"],
            },
        ),
        (
            "nested body followed else",
            {
                "body": [{"try": ["invalid"]}, {"else": ["bar"]}],
                "else_bodies": [],
                "expected_packages": ["bar"],
            },
        ),
        (
            "nested else followed body",
            {
                "body": ["invalid"],
                "else_bodies": [[{"try": ["foo"]}, {"else": ["bar"]}]],
                "expected_packages": ["foo"],
            },
        ),
        (
            "nested else followed else",
            {
                "body": ["invalid"],
                "else_bodies": [[{"try": ["invalid"]}, {"else": ["bar"]}]],
                "expected_packages": ["bar"],
            },
        ),
        (
            "multiple elses",
            {
                "body": ["invalid1"],
                "else_bodies": [["invalid2"], ["valid"]],
                "expected_packages": ["valid"],
            },
        ),
        (
            "multiple elses all invalid",
            {
                "body": ["invalid1"],
                "else_bodies": [["invalid2"], ["invalid3"]],
                "expected_packages": ["invalid3"],
            },
        ),
    ]

    def test_try_statement_grammar(self, body, else_bodies, expected_packages):
        processor = grammar.GrammarProcessor(
            None, snapcraft.ProjectOptions(), lambda x: "invalid" not in x
        )
        statement = _try.TryStatement(body=body, processor=processor)

        for else_body in else_bodies:
            statement.add_else(else_body)

        assert statement.process() == expected_packages


def test_else_fail():
    processor = grammar.GrammarProcessor(
        None, snapcraft.ProjectOptions(), lambda x: "invalid" not in x
    )
    statement = _try.TryStatement(body=["invalid"], processor=processor)

    statement.add_else(None)

    with pytest.raises(grammar.errors.UnsatisfiedStatementError) as error:
        statement.process()

    assert "Unable to satisfy 'try', failure forced" in str(error.value)
