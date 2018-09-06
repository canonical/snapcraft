# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import contextlib
import copy
import jsonschema
import importlib
import logging
from typing import Any, Dict, List, Set, Type  # noqa: F401

from snapcraft import formatting_utils
from .. import errors
from ._extension import Extension

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
    base = yaml_data.get("base")

    applied_extension_names = set()  # type: Set[str]
    global_extension_names = yaml_data.get("extensions", [])
    _validate_extension_format(global_extension_names)

    for app_name, app_definition in yaml_data.get("apps", dict()).items():
        extension_names = app_definition.get("extensions")
        _validate_extension_format(extension_names)

        # Make sure global extensions are assigned to any app without extensions
        if extension_names is None:
            extension_names = global_extension_names

        for extension_name in extension_names:
            extension = _load_extension(base, extension_name, yaml_data)
            _apply_extension(yaml_data, app_name, extension_name, extension)

        # Keep track of the extensions applied so we can warn about any that
        # are declared, but not used
        applied_extension_names.update(extension_names)

        # Now that extensions have been applied, remove the specification from
        # this app
        with contextlib.suppress(KeyError):
            del yaml_data["apps"][app_name]["extensions"]

    # Now that extensions have been applied, remove the global specification
    with contextlib.suppress(KeyError):
        del yaml_data["extensions"]

    unused_extensions = set(global_extension_names) - applied_extension_names
    if unused_extensions:
        logger.warning(
            "The following extensions are declared, but not used: {}".format(
                formatting_utils.humanize_list(unused_extensions, "and")
            )
        )

    return yaml_data


def find_extension(extension_name: str) -> Type[Extension]:
    """Find and return the extension class with the given name.

    :param str extension_name: The name of the extension to load

    :returns: Extension subclass responsible for extension.
    :rtype: type(Extension)
    :raises: errors.ExtensionNotFoundError if the extension is not found
    """
    try:
        extension_module = importlib.import_module(
            "snapcraft.internal.project_loader._extensions.{}".format(extension_name)
        )
    except ImportError:
        raise errors.ExtensionNotFoundError(extension_name)

    # This may throw an AttributeError, but that would be programmer error of whoever
    # is hacking on extensions.
    extension_class_name = "{}Extension".format(extension_name.capitalize())
    return getattr(extension_module, extension_class_name)


def _load_extension(base: str, extension_name: str, yaml_data) -> Extension:
    # A base is required in order to use extensions, so raise an error if not specified.
    if not base:
        raise errors.ExtensionBaseRequiredError()

    extension_class = find_extension(extension_name)
    if base not in extension_class.supported_bases:
        raise errors.ExtensionUnsupportedBaseError(extension_name, base)

    # Hand the extension a copy of the yaml data so the only way they can modify it is
    # by going through the extension API.
    return extension_class(copy.deepcopy(yaml_data))


def _apply_extension(
    yaml_data: Dict[str, Any], app_name: str, extension_name: str, extension: Extension
):
    # Apply the app-specific components of the extension (if any)
    app_extension = extension.app_snippet
    app_definition = yaml_data["apps"][app_name]
    for property_name, property_value in app_extension.items():
        app_definition[property_name] = _apply_extension_property(
            app_definition.get(property_name), property_value
        )

    # Next, apply the part-specific components
    part_extension = extension.part_snippet
    parts = yaml_data["parts"]
    for part_name, part_definition in parts.items():
        for property_name, property_value in part_extension.items():
            part_definition[property_name] = _apply_extension_property(
                part_definition.get(property_name), property_value
            )

    # Finally, add any parts specified in the extension
    for part_name, part_definition in extension.parts.items():
        # If a extension part name clashes with a part that already exists, error.
        if part_name in parts:
            raise errors.ExtensionPartConflictError(extension_name, part_name)

        parts[part_name] = part_definition


def _apply_extension_property(existing_property: Any, extension_property: Any):
    if existing_property:
        # If the property is not scalar, merge them
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
        return existing_property

    return extension_property


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
            raise errors.YamlValidationError(
                "The 'extensions' property does not match the required schema: {}".format(
                    errors.YamlValidationError.from_validation_error(e).message
                )
            )
