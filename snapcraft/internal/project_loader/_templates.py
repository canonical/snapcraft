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
import functools
import jsonschema
import logging
import os
from typing import Any, Dict, List, Set  # noqa: F401
import yaml

from snapcraft import formatting_utils
from snapcraft.internal import common
from . import errors

logger = logging.getLogger(__name__)

# Templates are evaluated before the snapcraft.yaml is validated, so we need to handle
# our own subset of validation ourselves (covering only the usage of templates).
template_schema = {
    "type": "array",
    "minitems": 1,
    "uniqueItems": True,
    "items": {"type": "string"},
}


def apply_templates(yaml_data: Dict[str, Any]) -> Dict[str, Any]:
    """Apply all templates.

    :param dict yaml_data: Loaded, unprocessed snapcraft.yaml
    :returns: Modified snapcraft.yaml data with templates applied
    """
    # Don't modify the dict passed in
    yaml_data = copy.deepcopy(yaml_data)
    base = yaml_data.get("base")

    applied_template_names = set()  # type: Set[str]
    global_template_names = yaml_data.get("templates", [])
    _validate_template_format(global_template_names)

    for app_name, app_definition in yaml_data.get("apps", dict()).items():
        template_names = app_definition.get("templates")
        _validate_template_format(template_names)

        # Make sure global templates are assigned to any app without templates
        if template_names is None:
            template_names = global_template_names

        for template_name in template_names:
            template_data = _find_template(base, template_name)
            _apply_template(yaml_data, app_name, template_name, template_data)

        # Keep track of the templates applied so we can warn about any that
        # are declared, but not used
        applied_template_names.update(template_names)

        # Now that templates have been applied, remove the specification from
        # this app
        with contextlib.suppress(KeyError):
            del yaml_data["apps"][app_name]["templates"]

    # Now that templates have been applied, remove the global specification
    with contextlib.suppress(KeyError):
        del yaml_data["templates"]

    unused_templates = set(global_template_names) - applied_template_names
    if unused_templates:
        logger.warning(
            "The following templates are declared, but not used: {}".format(
                formatting_utils.humanize_list(unused_templates, "and")
            )
        )

    return yaml_data


def template_yaml_path(template_name: str) -> str:
    """Return the file path to the template's template.yaml

    :param str template_name: The name of the template
    :returns: File path to the template.yaml
    :raises: errors.TemplateNotFoundError if the template is not found
    """
    template_yaml_path = os.path.join(
        common.get_templatesdir(), template_name, "template.yaml"
    )

    if not os.path.isdir(os.path.dirname(template_yaml_path)):
        raise errors.TemplateNotFoundError(template_name)

    return template_yaml_path


def load_template(template_name: str) -> Dict[str, Any]:
    """Load and return the template with the given name.

    :param str template_name: The name of the template to load
    :raises: errors.TemplateNotFoundError if the template is not found
    """
    return copy.deepcopy(__template_loader(template_name))


def _find_template(base: str, template_name: str) -> Dict[str, Any]:
    # A base is required in order to use templates, so raise an error if not specified.
    if not base:
        raise errors.TemplateBaseRequiredError()

    try:
        return load_template(template_name)[base]
    except KeyError:
        raise errors.TemplateUnsupportedBaseError(template_name, base)


# Don't load the same template multiple times
@functools.lru_cache()
def __template_loader(template_name: str) -> Dict[str, Any]:
    with open(template_yaml_path(template_name), "r") as f:
        return yaml.safe_load(f)


def _apply_template(
    yaml_data: Dict[str, Any],
    app_name: str,
    template_name: str,
    template_data: Dict[str, Any],
):
    # Apply the app-specific components of the template (if any)
    template_app_components = template_data.pop("apps", None)
    if template_app_components:
        app_template = template_app_components.pop("*", {})

        # If there are any other app components, a template developer is trying to add
        # an app. Try to be helpful.
        if template_app_components:
            raise RuntimeError(
                "The 'apps' section of templates should contain no more than '*'. "
                "Adding standalone apps from templates is not currently supported."
            )

        app_definition = yaml_data["apps"][app_name]
        for property_name, property_value in app_template.items():
            app_definition[property_name] = _apply_template_property(
                app_definition.get(property_name), property_value
            )

    # Next, apply the part-specific components
    template_part_components = template_data.pop("parts", None)
    if template_part_components:
        part_template = template_part_components.pop("*", {})

        parts = yaml_data["parts"]
        for part_name, part_definition in parts.items():
            for property_name, property_value in part_template.items():
                part_definition[property_name] = _apply_template_property(
                    part_definition.get(property_name), property_value
                )

        # Finally, add any parts specified in the template
        for part_name, part_definition in template_part_components.items():
            # If a template part name clashes with a part that already exists, error.
            if part_name in parts:
                raise errors.TemplatePartConflictError(template_name, part_name)

            parts[part_name] = part_definition

    # If there is anything left in the template, a template developer is trying to do
    # something that isn't supported. Try to be helpful.
    if template_data:
        raise RuntimeError(
            "Templates are only capable of specifying 'apps' and 'parts'. {} keys are "
            "currently unsupported.".format(
                formatting_utils.humanize_list(template_data.keys(), "and")
            )
        )


def _apply_template_property(existing_property: Any, template_property: Any):
    if existing_property:
        if type(existing_property) is type(template_property):
            # If the property is not scalar, merge them
            if isinstance(existing_property, list):
                return _merge_lists(existing_property, template_property)
            elif isinstance(existing_property, dict):
                for key, value in template_property.items():
                    existing_property[key] = _apply_template_property(
                        existing_property.get(key), value
                    )
                return existing_property
        return existing_property

    return template_property


def _merge_lists(list1: List[str], list2: List[str]) -> List[str]:
    """Merge two lists while maintaining order and removing duplicates."""
    seen = set()  # type: Set[str]
    merged = list()  # type: List[str]

    for item in list1 + list2:
        if item not in seen:
            seen.add(item)
            merged.append(item)

    return merged


def _validate_template_format(template_names):
    if template_names is not None:
        format_check = jsonschema.FormatChecker()
        try:
            jsonschema.validate(
                template_names, template_schema, format_checker=format_check
            )
        except jsonschema.ValidationError as e:
            raise errors.YamlValidationError(
                "The 'templates' property does not match the required schema: {}".format(
                    errors.YamlValidationError.from_validation_error(e).message
                )
            )
