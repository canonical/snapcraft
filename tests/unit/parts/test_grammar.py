# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Grammar processor tests."""

from collections import namedtuple

import pytest
from craft_grammar import GrammarProcessor

from snapcraft.parts.grammar import process_part, process_parts

_PROCESSOR = GrammarProcessor(
    arch="amd64",
    target_arch="amd64",
    checker=lambda x: x == x,  # pylint: disable=comparison-with-itself  # noqa PLR0124
)
GrammarEntry = namedtuple("GrammarEntry", ["value", "expected"])

GRAMMAR_SCALAR_ENTRIES = [
    # no grammar.
    GrammarEntry("entry", "entry"),
    # on arch match.
    GrammarEntry([{"on amd64": "entry"}], "entry"),
    # on else match.
    GrammarEntry([{"on arm64": "entry"}, {"else": "else-entry"}], "else-entry"),
    # on other-arch no else.
    GrammarEntry([{"on arm64": "entry"}], None),
    # TODO: on <arch> to <target-arch> match
]


@pytest.mark.parametrize("grammar_entry", GRAMMAR_SCALAR_ENTRIES)
@pytest.mark.parametrize("key", ["source"])
def test_scalar_values(key, grammar_entry):
    part_yaml_data = {key: grammar_entry.value}

    value = process_part(part_yaml_data=part_yaml_data, processor=_PROCESSOR)

    expected = {key: grammar_entry.expected}
    assert value == expected


GRAMMAR_LIST_ENTRIES = [
    # no grammar.
    GrammarEntry(["entry"], ["entry"]),
    # on arch match.
    GrammarEntry([{"on amd64": ["entry"]}], ["entry"]),
    # on else match.
    GrammarEntry([{"on arm64": ["entry"]}, {"else": ["else-entry"]}], ["else-entry"]),
    # on other-arch no else.
    GrammarEntry([{"on arm64": ["entry"]}], []),
    # TODO: on <arch> to <target-arch> match
]


@pytest.mark.parametrize("grammar_entry", GRAMMAR_LIST_ENTRIES)
@pytest.mark.parametrize(
    "key",
    [
        "build-environment",
        "build-packages",
        "stage-packages",
        "build-snaps",
        "stage-snaps",
    ],
)
def test_list_values(key, grammar_entry):
    part_yaml_data = {key: grammar_entry.value}

    value = process_part(part_yaml_data=part_yaml_data, processor=_PROCESSOR)

    expected = {key: grammar_entry.expected}
    assert value == expected


def test_process_grammar():
    assert process_parts(
        parts_yaml_data={
            "no-grammar": {
                "source": "source-foo",
                "build-environment": ["env-foo"],
                "build-packages": ["build-pkg-foo"],
                "stage-packages": ["stage-pkg-foo"],
                "build-snaps": ["build-snap-foo"],
                "stage-snaps": ["stage-snap-foo"],
            },
            "grammar": {
                "source": [
                    {
                        "on amd64": "source-foo",
                    },
                ],
                "build-environment": [
                    {
                        "on amd64": ["env-foo"],
                    },
                ],
                "build-packages": [
                    {
                        "on amd64": ["build-pkg-foo"],
                    },
                ],
                "stage-packages": [
                    {
                        "on amd64": ["stage-pkg-foo"],
                    },
                ],
                "build-snaps": [
                    {
                        "on amd64": ["build-snap-foo"],
                    },
                ],
                "stage-snaps": [
                    {
                        "on amd64": ["stage-snap-foo"],
                    },
                ],
            },
        },
        arch="amd64",
        target_arch="amd64",
    ) == {
        "no-grammar": {
            "source": "source-foo",
            "build-environment": ["env-foo"],
            "build-packages": ["build-pkg-foo"],
            "stage-packages": ["stage-pkg-foo"],
            "build-snaps": ["build-snap-foo"],
            "stage-snaps": ["stage-snap-foo"],
        },
        "grammar": {
            "source": "source-foo",
            "build-environment": ["env-foo"],
            "build-packages": ["build-pkg-foo"],
            "stage-packages": ["stage-pkg-foo"],
            "build-snaps": ["build-snap-foo"],
            "stage-snaps": ["stage-snap-foo"],
        },
    }
