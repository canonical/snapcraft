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

"""Grammar processor."""

from typing import Any, Dict

from craft_grammar import GrammarProcessor

_KEYS = [
    "source",
    "build-environment",
    "build-packages",
    "stage-packages",
    "build-snaps",
    "stage-snaps",
]

_SCALAR_VALUES = ["source"]


def process_part(
    *, part_yaml_data: Dict[str, Any], processor: GrammarProcessor
) -> Dict[str, Any]:
    """Process grammar for a given part."""
    existing_keys = (key for key in _KEYS if key in part_yaml_data)

    for key in existing_keys:
        unprocessed_grammar = part_yaml_data[key]

        if key in _SCALAR_VALUES and isinstance(unprocessed_grammar, str):
            unprocessed_grammar = [unprocessed_grammar]

        processed_grammar = processor.process(grammar=unprocessed_grammar)

        if key in _SCALAR_VALUES and isinstance(processed_grammar, list):
            if processed_grammar:
                processed_grammar = processed_grammar[0]
            else:
                processed_grammar = None
        part_yaml_data[key] = processed_grammar

    return part_yaml_data


def process_parts(
    *, parts_yaml_data: Dict[str, Any], arch: str, target_arch: str
) -> Dict[str, Any]:
    """Process grammar for parts.

    :param yaml_data: unprocessed snapcraft.yaml.
    :returns: process snapcraft.yaml.
    """
    # TODO: make checker optional in craft-grammar.
    processor = GrammarProcessor(
        arch=arch,
        target_arch=target_arch,
        checker=lambda x: x == x,  # pylint: disable=comparison-with-itself  # noqa PLR0124
    )

    for part_name in parts_yaml_data:
        parts_yaml_data[part_name] = process_part(
            part_yaml_data=parts_yaml_data[part_name], processor=processor
        )

    return parts_yaml_data
