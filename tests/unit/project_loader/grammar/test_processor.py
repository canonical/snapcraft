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

import platform
import re

import pytest

import snapcraft
import snapcraft.internal.project_loader.grammar._to as _to
from snapcraft.internal.project_loader import grammar


@pytest.mark.parametrize(
    "entry",
    [
        [{"on amd64,i386": ["foo"]}, {"on amd64,i386": ["bar"]}],
        [{"on amd64,i386": ["foo"]}, {"on i386,amd64": ["bar"]}],
    ],
)
def test_duplicates(entry):
    """Test that multiple identical selector sets is an error."""

    processor = grammar.GrammarProcessor(
        entry, snapcraft.ProjectOptions(), lambda x: True
    )
    with pytest.raises(grammar.errors.GrammarSyntaxError) as error:
        processor.process()

    expected = (
        "Invalid grammar syntax: found duplicate 'on amd64,i386' "
        "statements. These should be merged."
    )
    assert expected in str(error.value)


class TestBasicGrammar:

    scenarios = [
        (
            "unconditional",
            {
                "grammar_entry": ["foo", "bar"],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"foo", "bar"},
            },
        ),
        (
            "mixed including",
            {
                "grammar_entry": ["foo", {"on i386": ["bar"]}],
                "host_arch": "i686",
                "target_arch": "i386",
                "expected_packages": {"foo", "bar"},
            },
        ),
        (
            "mixed excluding",
            {
                "grammar_entry": ["foo", {"on i386": ["bar"]}],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "on amd64",
            {
                "grammar_entry": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "on i386",
            {
                "grammar_entry": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}],
                "host_arch": "i686",
                "target_arch": "i386",
                "expected_packages": {"bar"},
            },
        ),
        (
            "ignored else",
            {
                "grammar_entry": [{"on amd64": ["foo"]}, {"else": ["bar"]}],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "used else",
            {
                "grammar_entry": [{"on amd64": ["foo"]}, {"else": ["bar"]}],
                "host_arch": "i686",
                "target_arch": "i386",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested amd64",
            {
                "grammar_entry": [
                    {"on amd64": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}]}
                ],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "nested i386",
            {
                "grammar_entry": [
                    {"on i386": [{"on amd64": ["foo"]}, {"on i386": ["bar"]}]}
                ],
                "host_arch": "i686",
                "target_arch": "i386",
                "expected_packages": {"bar"},
            },
        ),
        (
            "nested ignored else",
            {
                "grammar_entry": [
                    {"on amd64": [{"on amd64": ["foo"]}, {"else": ["bar"]}]}
                ],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "nested used else",
            {
                "grammar_entry": [
                    {"on i386": [{"on amd64": ["foo"]}, {"else": ["bar"]}]}
                ],
                "host_arch": "i686",
                "target_arch": "amd64",
                "expected_packages": {"bar"},
            },
        ),
        (
            "try",
            {
                "grammar_entry": [{"try": ["valid"]}],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"valid"},
            },
        ),
        (
            "try else",
            {
                "grammar_entry": [{"try": ["invalid"]}, {"else": ["valid"]}],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"valid"},
            },
        ),
        (
            "nested try",
            {
                "grammar_entry": [{"on amd64": [{"try": ["foo"]}, {"else": ["bar"]}]}],
                "host_arch": "x86_64",
                "target_arch": "amd64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "nested try else",
            {
                "grammar_entry": [
                    {"on i386": [{"try": ["invalid"]}, {"else": ["bar"]}]}
                ],
                "host_arch": "i686",
                "target_arch": "i686",
                "expected_packages": {"bar"},
            },
        ),
        (
            "optional",
            {
                "grammar_entry": ["foo", {"try": ["invalid"]}],
                "host_arch": "i686",
                "target_arch": "i386",
                "expected_packages": {"foo"},
            },
        ),
        (
            "multi",
            {
                "grammar_entry": [
                    "foo",
                    {"on amd64": ["foo2"]},
                    {"on amd64 to arm64": ["foo3"]},
                ],
                "host_arch": "x86_64",
                "target_arch": "i386",
                "expected_packages": {"foo", "foo2"},
            },
        ),
    ]

    def test_basic_grammar(
        self, monkeypatch, grammar_entry, host_arch, target_arch, expected_packages
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        project = snapcraft.ProjectOptions(target_deb_arch=target_arch)

        processor = grammar.GrammarProcessor(
            grammar_entry, project, lambda x: "invalid" not in x
        )
        assert processor.process() == expected_packages


class TestTransformerGrammar:

    scenarios = [
        (
            "unconditional",
            {
                "grammar_entry": ["foo", "bar"],
                "host_arch": "x86_64",
                "expected_packages": {"foo", "bar"},
            },
        ),
        (
            "mixed including",
            {
                "grammar_entry": ["foo", {"on i386": ["bar"]}],
                "host_arch": "i686",
                "expected_packages": {"foo", "bar"},
            },
        ),
        (
            "mixed excluding",
            {
                "grammar_entry": ["foo", {"on i386": ["bar"]}],
                "host_arch": "x86_64",
                "expected_packages": {"foo"},
            },
        ),
        (
            "to",
            {
                "grammar_entry": [{"to i386": ["foo"]}],
                "host_arch": "x86_64",
                "expected_packages": {"foo:i386"},
            },
        ),
        (
            "transform applies to nested",
            {
                "grammar_entry": [{"to i386": [{"on amd64": ["foo"]}]}],
                "host_arch": "x86_64",
                "expected_packages": {"foo:i386"},
            },
        ),
        (
            "not to",
            {
                "grammar_entry": [{"to amd64": ["foo"]}, {"else": ["bar"]}],
                "host_arch": "x86_64",
                "expected_packages": {"bar"},
            },
        ),
    ]

    def test_grammar_with_transformer(
        self, monkeypatch, grammar_entry, host_arch, expected_packages
    ):
        monkeypatch.setattr(platform, "machine", lambda: host_arch)
        monkeypatch.setattr(platform, "architecture", lambda: ("64bit", "ELF"))

        # Transform all 'to' statements to include arch
        def _transformer(call_stack, package_name, project_options):
            if any(isinstance(s, _to.ToStatement) for s in call_stack):
                if ":" not in package_name:
                    package_name += ":{}".format(project_options.deb_arch)

            return package_name

        processor = grammar.GrammarProcessor(
            grammar_entry,
            snapcraft.ProjectOptions(target_deb_arch="i386"),
            lambda x: True,
            transformer=_transformer,
        )

        assert processor.process() == expected_packages


class TestInvalidGrammar:

    scenarios = [
        (
            "unmatched else",
            {
                "grammar_entry": [{"else": ["foo"]}],
                "expected_exception": ".*'else' doesn't seem to correspond.*",
            },
        ),
        (
            "unmatched else fail",
            {
                "grammar_entry": ["else fail"],
                "expected_exception": ".*'else' doesn't seem to correspond.*",
            },
        ),
        (
            "unexpected type",
            {
                "grammar_entry": [5],
                "expected_exception": ".*expected grammar section.*but got.*",
            },
        ),
    ]

    def test_invalid_grammar(self, grammar_entry, expected_exception):
        processor = grammar.GrammarProcessor(
            grammar_entry, snapcraft.ProjectOptions(), lambda x: True
        )

        with pytest.raises(grammar.errors.GrammarSyntaxError) as error:
            processor.process()

        assert re.match(expected_exception, str(error.value))
