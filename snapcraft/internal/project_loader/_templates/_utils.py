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
from ._template import Template

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
            template = _load_template(base, template_name, yaml_data)
            _apply_template(yaml_data, app_name, template_name, template)

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


def find_template(template_name: str) -> Type[Template]:
    """Find and return the template class with the given name.

    :param str template_name: The name of the template to load

    :returns: Template subclass responsible for template.
    :rtype: type(Template)
    :raises: errors.TemplateNotFoundError if the template is not found
    """
    try:
        template_module = importlib.import_module(
            "snapcraft.internal.project_loader._templates.{}".format(template_name)
        )
    except ImportError:
        raise errors.TemplateNotFoundError(template_name)

    # This may throw an AttributeError, but that would be programmer error of whoever
    # is hacking on templates.
    template_class_name = "{}Template".format(template_name.capitalize())
    return getattr(template_module, template_class_name)


def _load_template(base: str, template_name: str, yaml_data) -> Template:
    # A base is required in order to use templates, so raise an error if not specified.
    if not base:
        raise errors.TemplateBaseRequiredError()

    template_class = find_template(template_name)
    if base not in template_class.supported_bases:
        raise errors.TemplateUnsupportedBaseError(template_name, base)

    # Hand the template a copy of the yaml data so the only way they can modify it is
    # by going through the template API.
    return template_class(copy.deepcopy(yaml_data))


def _apply_template(
    yaml_data: Dict[str, Any], app_name: str, template_name: str, template: Template
):
    # Apply the app-specific components of the template (if any)
    app_template = template.app_snippet
    app_definition = yaml_data["apps"][app_name]
    for property_name, property_value in app_template.items():
        app_definition[property_name] = _apply_template_property(
            app_definition.get(property_name), property_value
        )

    # Next, apply the part-specific components
    part_template = template.part_snippet
    parts = yaml_data["parts"]
    for part_name, part_definition in parts.items():
        for property_name, property_value in part_template.items():
            part_definition[property_name] = _apply_template_property(
                part_definition.get(property_name), property_value
            )

    # Finally, add any parts specified in the template
    for part_name, part_definition in template.parts.items():
        # If a template part name clashes with a part that already exists, error.
        if part_name in parts:
            raise errors.TemplatePartConflictError(template_name, part_name)

        parts[part_name] = part_definition


def _apply_template_property(existing_property: Any, template_property: Any):
    if existing_property:
        # If the property is not scalar, merge them
        if isinstance(existing_property, list) and isinstance(template_property, list):
            return _merge_lists(existing_property, template_property)
        elif isinstance(existing_property, dict) and isinstance(
            template_property, dict
        ):
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
