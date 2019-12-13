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

import collections
from collections import OrderedDict
import contextlib
import copy
import jsonschema
import importlib
import logging
import os
import pkgutil
from typing import Any, Dict, List, Optional, Set, Type

from .. import errors
from ._extension import Extension
from snapcraft.project import errors as project_errors

logger = logging.getLogger(__name__)

# Extensions are evaluated before the snapcraft.yaml is validated, so we need to handle
# our own subset of validation ourselves (covering only the usage of extensions).
extension_schema = {
    "type": "array",
    "minitems": 1,
    "uniqueItems": True,
    "items": {"type": "string"},
}


def apply_extensions(yaml_data: Dict[str, Any]) -> Dict[str, Any]:
    """Apply all extensions.

    :param dict yaml_data: Loaded, unprocessed snapcraft.yaml
    :returns: Modified snapcraft.yaml data with extensions applied
    """
    # Don't modify the dict passed in
    yaml_data = copy.deepcopy(yaml_data)
    original_yaml_data = copy.deepcopy(yaml_data)
    base: Optional[str] = yaml_data.get("base")

    # Mapping of extension names to set of app names to which the extension needs to be
    # applied.
    declared_extensions = collections.defaultdict(set)  # type: Dict[str, Set[str]]

    for app_name, app_definition in yaml_data.get("apps", dict()).items():
        extension_names = app_definition.get("extensions", [])
        _validate_extension_format(extension_names)

        for extension_name in extension_names:
            declared_extensions[extension_name].add(app_name)

        # Now that we've saved the app -> extension relationship, remove the property
        # from this app's declaration in the YAML.
        with contextlib.suppress(KeyError):
            del yaml_data["apps"][app_name]["extensions"]

    # Process extensions in a consistent order.
    for extension_name in sorted(declared_extensions.keys()):
        extension = _load_extension(base, extension_name, original_yaml_data)
        _apply_extension(
            yaml_data, declared_extensions[extension_name], extension_name, extension
        )

    return yaml_data


def find_extension(extension_name: str) -> Type[Extension]:
    """Find and return the extension class with the given name.

    :param str extension_name: The name of the extension to load

    :returns: Extension subclass responsible for extension.
    :rtype: type(Extension)
    :raises: errors.ExtensionNotFoundError if the extension is not found
    """
    if extension_name.startswith("_"):
        raise errors.ExtensionNotFoundError(extension_name)

    try:
        extension_module = importlib.import_module(
            "snapcraft.internal.project_loader._extensions.{}".format(
                extension_name.replace("-", "_")
            )
        )
    except ImportError:
        raise errors.ExtensionNotFoundError(extension_name)

    # The extension module requires a class named ExtensionImpl.
    return getattr(extension_module, "ExtensionImpl")


def supported_extension_names() -> List[str]:
    """Return a list of supported extension names.

    :returns: List of supported extension names.
    :rtype: list
    """

    extension_names = []  # type: List[str]
    for _, modname, _ in pkgutil.iter_modules([os.path.dirname(__file__)]):
        # Only add non-private modules/packages to the extension list.
        if not modname.startswith("_"):
            extension_names.append(modname)

    return extension_names


def _load_extension(
    base: Optional[str], extension_name: str, yaml_data: Dict[str, Any]
) -> Extension:
    extension_class = find_extension(extension_name)

    # Hand the extension a copy of the yaml data so the only way they can modify it is
    # by going through the extension API.
    return extension_class(
        extension_name=extension_name, yaml_data=copy.deepcopy(yaml_data)
    )


def _convert_to_ordereddict(
    property_list: List[Dict[str, str]]
) -> OrderedDict:
    od = OrderedDict()
    for elem in property_list:
        if not isinstance(elem, dict) or len(elem) != 1:
            raise RuntimeError("Badly disguised ordered dictionary!")
        for k, v in elem.items():
            od[k] = v
    return od


def _convert_to_disguised_ordereddict(
    od: OrderedDict
) -> List[Dict[str, str]]:
    disguised: List[Dict[str, str]] = list()
    for k, v in od.items():
        disguised.append({k: v})
    return disguised


def _merge_build_environment(
    existing: List[Dict[str, str]], extension: List[Dict[str, str]]
) -> List[Dict[str, str]]:
    # build-environment is an OrderedDict in disguise.
    if not existing:
        return extension
    if not extension:
        return existing
    existing_dict = _convert_to_ordereddict(existing)
    extension_dict = _convert_to_ordereddict(extension)
    extension_dict.update(existing_dict)
    return _convert_to_disguised_ordereddict(extension_dict)


def _apply_extension(
    yaml_data: Dict[str, Any],
    app_names: Set[str],
    extension_name: str,
    extension: Extension,
):
    # Apply the root components of the extension (if any).
    root_extension = extension.root_snippet
    for property_name, property_value in root_extension.items():
        yaml_data[property_name] = _apply_extension_property(
            yaml_data.get(property_name), property_value
        )

    # Apply the app-specific components of the extension (if any).
    app_extension = extension.app_snippet
    for app_name in app_names:
        app_definition = yaml_data["apps"][app_name]
        for property_name, property_value in app_extension.items():
            app_definition[property_name] = _apply_extension_property(
                app_definition.get(property_name), property_value
            )

    # Next, apply the part-specific components.
    part_extension = extension.part_snippet
    parts = yaml_data["parts"]
    for part_name, part_definition in parts.items():

        for property_name, property_value in part_extension.items():
            if property_name == "build-environment":
                part_definition[property_name] = _merge_build_environment(
                    part_definition.get(property_name), property_value
                )
            else:
                part_definition[property_name] = _apply_extension_property(
                    part_definition.get(property_name), property_value
                )

        # Stores the extension's list of part_snippets in each part.
        parts[part_name] = part_definition

    # Finally, add any parts specified in the extension.
    for part_name, part_definition in extension.parts.items():
        # If a extension part name clashes with a part that already exists, error.
        if part_name in parts:
            raise errors.ExtensionPartConflictError(extension_name, part_name)

        # Stores the extension's list of parts in the parts section.
        parts[part_name] = part_definition


def _apply_extension_property(existing_property: Any, extension_property: Any) -> Dict[str, Any]:
    """Take the user-defined yaml properties and apply any missing
    extension properties. If there is a property defined in both, 
    the user-defined property is applied.
    """

    # If there is no user-defined property, then just add the extension-defined property. 
    if not existing_property:
        return extension_property

    # If there is a user-defined property, then take care when merging with the extension-defined property.
    if isinstance(existing_property, list) and isinstance(extension_property, list):
        return _merge_lists(existing_property, extension_property)

    elif isinstance(existing_property, dict) and isinstance(
        extension_property, dict
    ):
        for key, value in extension_property.items():
            existing_property[key] = _apply_extension_property(
                existing_property.get(key), value
            )
    return existing_property


def _merge_lists(list1: List[str], list2: List[str]) -> List[str]:
    """Merge two lists while maintaining order and removing duplicates."""

    seen = set()  # type: Set[str]
    merged = list()  # type: List[str]

    for item in list1 + list2:
        if item not in seen:
            seen.add(item)
            merged.append(item)

    return merged


def _validate_extension_format(extension_names):
    if extension_names is not None:
        format_check = jsonschema.FormatChecker()
        try:
            jsonschema.validate(
                extension_names, extension_schema, format_checker=format_check
            )
        except jsonschema.ValidationError as e:
            raise project_errors.YamlValidationError(
                "The 'extensions' property does not match the required schema: {}".format(
                    project_errors.YamlValidationError.from_validation_error(e).message
                )
            )
