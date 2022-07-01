# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

"""Contains the configuration pre-processor."""

from typing import Any, Dict, List

from snapcraft_legacy.yaml_utils.errors import YamlValidationError


def preprocessor(configuration: Dict[str, Any]) -> Dict[str, Any]:
    """Apply all the preprocessors to the configuration file."""
    return _manage_groups(configuration)


def _manage_groups(configuration: Dict[str, Any]) -> Dict[str, Any]:
    """Manage the 'groups' entry.

    :param configuration: the configuration as processed by the YAML module.
    :return: the modified configuration.
    """
    if "parts" in configuration:
        _check_groups_syntax(configuration)

        groups_list: Dict[str, List] = {}

        for part in configuration["parts"]:
            data = configuration["parts"][part]
            if "groups" not in data:
                continue
            for group in data["groups"]:
                if group not in groups_list:
                    groups_list[group] = []
                groups_list[group].append(part)

            # remove the 'groups' key because the rest of the code doesn't understand it
            del data["groups"]

        for part in configuration["parts"]:
            data = configuration["parts"][part]
            if "after" not in data:
                continue

            new_deps = []
            for dependency in data["after"]:
                if dependency not in groups_list:
                    new_deps.append(dependency)
                    continue
                new_deps.extend(groups_list[dependency])

            # apply the new dependencies removing duplicates
            data["after"] = list(dict.fromkeys(new_deps))

    return configuration


def _check_groups_syntax(configuration: Dict[str, Any]):
    """Check the syntax in the 'groups' elements.

    :param configuration: the configuration as processed by the YAML module.

    :raises YamlValidationError: if there are syntax errors.
    """
    for part in configuration["parts"]:
        data = configuration["parts"][part]
        if "groups" not in data:
            continue

        if not isinstance(data["groups"], list):
            raise YamlValidationError(
                f"The group entry in part '{part}' is not a list but"
                f"a {type(data['groups'])}. Aborting."
            )
        for group in data["groups"]:
            if not isinstance(group, str):
                raise YamlValidationError(
                    "There are entries in 'groups' in the part"
                    f"'{part}' that aren't strings. Aborting."
                )
            if group in configuration["parts"]:
                raise YamlValidationError(
                    f"The group {group} is already used as a part name. Aborting."
                )
