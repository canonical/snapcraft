# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

"""YAML utilities for Snapcraft."""

from typing import Any, Dict, TextIO

import yaml
import yaml.error

from snapcraft import errors, utils

_LEGACY_BASES = {"core", "core18", "core20"}


def _check_duplicate_keys(node):
    mappings = set()

    for key_node, _ in node.value:
        try:
            if key_node.value in mappings:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    f"found duplicate key {key_node.value!r}",
                    node.start_mark,
                )
            mappings.add(key_node.value)
        except TypeError:
            # Ignore errors for malformed inputs that will be caught later.
            pass


def _dict_constructor(loader, node):
    _check_duplicate_keys(node)

    # Necessary in order to make yaml merge tags work
    loader.flatten_mapping(node)
    value = loader.construct_pairs(node)

    try:
        return dict(value)
    except TypeError as type_error:
        raise yaml.constructor.ConstructorError(
            "while constructing a mapping",
            node.start_mark,
            "found unhashable key",
            node.start_mark,
        ) from type_error


class _SafeLoader(yaml.SafeLoader):  # pylint: disable=too-many-ancestors
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.add_constructor(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _dict_constructor
        )


def load(filestream: TextIO) -> Dict[str, Any]:
    """Load and parse a YAML-formatted file.

    :param filename: The YAML file to load.

    :raises SnapcraftError: if loading didn't succeed.
    :raises LegacyFallback: if the project's base is a legacy base.
    """
    try:
        data = yaml.safe_load(filestream)
        build_base = utils.get_effective_base(
            base=data.get("base"),
            build_base=data.get("build-base"),
            project_type=data.get("type"),
            name=data.get("name"),
        )

        if build_base is None:
            raise errors.LegacyFallback("no base defined")
        if build_base in _LEGACY_BASES:
            raise errors.LegacyFallback(f"base is {build_base}")
    except yaml.error.YAMLError as err:
        raise errors.SnapcraftError(f"snapcraft.yaml parsing error: {err!s}") from err

    filestream.seek(0)

    try:
        return yaml.load(
            filestream,
            Loader=_SafeLoader,  # noqa: S506 Probable unsafe use of yaml.load()
        )
    except yaml.error.YAMLError as err:
        raise errors.SnapcraftError(f"snapcraft.yaml parsing error: {err!s}") from err
