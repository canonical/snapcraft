# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import importlib
import logging
import sys

import jsonschema

import snapcraft
from snapcraft.internal.project_loader.errors import YamlValidationError
from snapcraft.internal import errors, sources

logger = logging.getLogger(__name__)


def load_plugin(
    plugin_name, part_name, project_options, properties, part_schema, definitions_schema
):
    module_name = plugin_name.replace("-", "_")
    module = _load_module(module_name, plugin_name, project_options)
    plugin_class = _get_plugin(module)
    if not plugin_class:
        raise errors.PluginError("no plugin found in module {!r}".format(plugin_name))

    _validate_pull_and_build_properties(
        plugin_name, plugin_class, part_schema, definitions_schema
    )

    try:
        options = _make_options(
            part_schema, definitions_schema, properties, plugin_class.schema()
        )
    except jsonschema.ValidationError as e:
        error = YamlValidationError.from_validation_error(e)
        raise errors.PluginError(
            "properties failed to load for {}: {}".format(part_name, error.message)
        )

    # For backwards compatibility we add the project to the plugin
    try:
        plugin = plugin_class(part_name, options, project_options)
    except TypeError:
        logger.warning(
            "DEPRECATED: the plugin used by part {!r} needs to be updated "
            "to accept project options in its initializer. See "
            "https://github.com/snapcore/snapcraft/blob/master/docs/"
            "plugins.md#initializing-a-plugin for more information".format(part_name)
        )
        plugin = plugin_class(part_name, options)
        # This is for plugins that don't inherit from BasePlugin
        if not hasattr(plugin, "project"):
            setattr(plugin, "project", project_options)
        # This is for plugins that inherit from BasePlugin but don't have
        # project in init.
        if not plugin.project:
            plugin.project = project_options

    if project_options.is_cross_compiling:
        logger.debug(
            "Setting {!r} as the compilation target for {!r}".format(
                project_options.deb_arch, plugin_name
            )
        )
        plugin.enable_cross_compilation()

    return plugin


def _load_module(module_name, plugin_name, project_options):
    module = None
    with contextlib.suppress(ImportError):
        module = _load_local(
            "x-{}".format(plugin_name), project_options.local_plugins_dir
        )
        logger.info("Loaded local plugin for %s", plugin_name)

    if not module:
        with contextlib.suppress(ImportError):
            module = importlib.import_module("snapcraft.plugins.{}".format(module_name))

    if not module:
        logger.info("Searching for local plugin for %s", plugin_name)
        with contextlib.suppress(ImportError):
            module = _load_local(module_name, project_options.local_plugins_dir)
        if not module:
            raise errors.PluginError("unknown plugin: {!r}".format(plugin_name))

    return module


def _load_local(module_name, local_plugin_dir):
    sys.path = [local_plugin_dir] + sys.path
    try:
        module = importlib.import_module(module_name)
    finally:
        sys.path.pop(0)

    return module


def _get_plugin(module):
    for attr in vars(module).values():
        if not isinstance(attr, type):
            continue
        if not issubclass(attr, snapcraft.BasePlugin):
            continue
        if attr == snapcraft.BasePlugin:
            continue
        return attr


def _validate_pull_and_build_properties(
    plugin_name, plugin, part_schema, definitions_schema
):
    merged_schema = _merged_part_and_plugin_schemas(
        part_schema, definitions_schema, plugin.schema()
    )
    merged_properties = merged_schema["properties"]

    # First, validate pull properties
    invalid_properties = _validate_step_properties(
        plugin.get_pull_properties(), merged_properties
    )

    if invalid_properties:
        raise errors.InvalidPullPropertiesError(plugin_name, list(invalid_properties))

    # Now, validate build properties
    invalid_properties = _validate_step_properties(
        plugin.get_build_properties(), merged_properties
    )

    if invalid_properties:
        raise errors.InvalidBuildPropertiesError(plugin_name, list(invalid_properties))


def _validate_step_properties(step_properties, schema_properties):
    invalid_properties = set()
    for step_property in step_properties:
        if step_property not in schema_properties:
            invalid_properties.add(step_property)

    return invalid_properties


def _make_options(part_schema, definitions_schema, properties, plugin_schema):
    # Make copies as these dictionaries are tampered with
    part_schema = part_schema.copy()
    properties = properties.copy()

    plugin_schema = _merged_part_and_plugin_schemas(
        part_schema, definitions_schema, plugin_schema
    )

    # This is for backwards compatibility for when most of the
    # schema was overridable by the plugins.
    if "required" in plugin_schema and not plugin_schema["required"]:
        del plugin_schema["required"]
    # With the same backwards compatibility in mind we need to remove
    # the source entry before validation. To those concerned, it has
    # already been validated.
    validated_properties = properties.copy()
    remove_set = [
        k for k in sources.get_source_defaults().keys() if k in validated_properties
    ]
    for key in remove_set:
        del validated_properties[key]

    jsonschema.validate(validated_properties, plugin_schema)

    options = _populate_options(properties, plugin_schema)

    return options


def _merged_part_and_plugin_schemas(part_schema, definitions_schema, plugin_schema):
    plugin_schema = plugin_schema.copy()
    if "properties" not in plugin_schema:
        plugin_schema["properties"] = {}

    if "definitions" not in plugin_schema:
        plugin_schema["definitions"] = {}

    # The part schema takes precedence over the plugin's schema.
    plugin_schema["properties"].update(part_schema)
    plugin_schema["definitions"].update(definitions_schema)

    return plugin_schema


def _populate_options(properties, schema):
    class Options:
        pass

    options = Options()

    schema_properties = schema.get("properties", {})
    for key in schema_properties:
        attr_name = key.replace("-", "_")
        default_value = schema_properties[key].get("default")
        attr_value = properties.get(key, default_value)
        setattr(options, attr_name, attr_value)

    return options
