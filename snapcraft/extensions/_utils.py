# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2022 Canonical Ltd.
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

"""Extension application helpers."""

import collections
import contextlib
import copy
from typing import Any, Dict, List, Set, TYPE_CHECKING

from .registry import get_extension_class


if TYPE_CHECKING:
    from .extension import Extension


def apply_extensions(
    yaml_data: Dict[str, Any], *, arch: str, target_arch: str
) -> Dict[str, Any]:
    """Apply all extensions.

    :param dict yaml_data: Loaded, unprocessed snapcraft.yaml
    :param arch: the host architecture.
    :param target_arch: the target architecture.
    :returns: Modified snapcraft.yaml data with extensions applied
    """
    # Don't modify the dict passed in
    yaml_data = copy.deepcopy(yaml_data)

    # Mapping of extension names to set of app names to which the extension needs to be
    # applied.
    declared_extensions: Dict[str, Set[str]] = collections.defaultdict(set)

    for app_name, app_definition in yaml_data.get("apps", {}).items():
        extension_names = app_definition.get("extensions", [])

        for extension_name in extension_names:
            declared_extensions[extension_name].add(app_name)

        # Now that we've saved the app -> extension relationship, remove the property
        # from this app's declaration in the YAML.
        with contextlib.suppress(KeyError):
            del yaml_data["apps"][app_name]["extensions"]

    # Process extensions in a consistent order
    for extension_name in sorted(declared_extensions.keys()):
        extension_class = get_extension_class(extension_name)
        extension = extension_class(
            yaml_data=copy.deepcopy(yaml_data), arch=arch, target_arch=target_arch
        )
        extension.validate(extension_name=extension_name)
        _apply_extension(yaml_data, declared_extensions[extension_name], extension)

    return yaml_data


def _apply_extension(
    yaml_data: Dict[str, Any],
    app_names: Set[str],
    extension: "Extension",
) -> None:
    # Apply the root components of the extension (if any)
    root_extension = extension.get_root_snippet()
    for property_name, property_value in root_extension.items():
        yaml_data[property_name] = _apply_extension_property(
            yaml_data.get(property_name), property_value
        )

    # Apply the app-specific components of the extension (if any)
    app_extension = extension.get_app_snippet()
    for app_name in app_names:
        app_definition = yaml_data["apps"][app_name]
        for property_name, property_value in app_extension.items():
            app_definition[property_name] = _apply_extension_property(
                app_definition.get(property_name), property_value
            )

    # Next, apply the part-specific components
    part_extension = extension.get_part_snippet()
    parts = yaml_data["parts"]
    for part_name, part_definition in parts.items():
        for property_name, property_value in part_extension.items():
            part_definition[property_name] = _apply_extension_property(
                part_definition.get(property_name), property_value
            )

    # Finally, add any parts specified in the extension
    for part_name, part_definition in extension.get_parts_snippet().items():
        parts[part_name] = part_definition


def _apply_extension_property(existing_property: Any, extension_property: Any) -> Any:
    if existing_property:
        # If the property is not scalar, merge them
        if isinstance(existing_property, list) and isinstance(extension_property, list):
            merged = extension_property + existing_property

            # If the lists are just strings, remove duplicates.
            if all(isinstance(item, str) for item in merged):
                return _remove_list_duplicates(merged)

            return merged

        if isinstance(existing_property, dict) and isinstance(extension_property, dict):
            for key, value in extension_property.items():
                existing_property[key] = _apply_extension_property(
                    existing_property.get(key), value
                )
            return existing_property
        return existing_property

    return extension_property


def _remove_list_duplicates(seq: List[str]) -> List[str]:
    """De-dupe string list maintaining ordering."""
    seen: Set[str] = set()
    deduped: List[str] = []

    for item in seq:
        if item not in seen:
            seen.add(item)
            deduped.append(item)

    return deduped
