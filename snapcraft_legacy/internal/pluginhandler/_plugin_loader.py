# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017,2020 Canonical Ltd
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
from pathlib import Path

import jsonschema

import snapcraft_legacy.yaml_utils.errors
from snapcraft_legacy import plugins
from snapcraft_legacy.internal import errors
from snapcraft_legacy.project import Project

logger = logging.getLogger(__name__)


def load_plugin(
    plugin_name: str,
    part_name: str,
    project: Project,
    properties,
    part_schema,
    definitions_schema,
) -> plugins.v2.PluginV2:
    local_plugins_dir = project._get_local_plugins_dir()

    plugin_class = _get_local_plugin_class(
        plugin_name=plugin_name, local_plugins_dir=Path(local_plugins_dir)
    )
    if plugin_class is None:
        plugin_class = plugins.get_plugin_for_base(
            plugin_name, build_base=project._get_build_base()
        )

    plugin_schema = plugin_class.get_schema()
    options = _make_options(
        part_name, part_schema, definitions_schema, properties, plugin_schema
    )
    plugin = plugin_class(part_name=part_name, options=options)

    return plugin


def _load_local(plugin_name: str, local_plugin_dir: Path):
    # The legacy module path is for the case when we allowed the plugin
    # file name to have '-', this is the first entry
    module_names = (
        Path(f"x-{plugin_name}.py"),
        Path(f"{plugin_name.replace('-', '_')}.py"),
    )
    module_paths = (local_plugin_dir / m for m in module_names)
    valid_paths = [m for m in module_paths if m.exists()]
    # No valid paths means no local plugin to load
    if not valid_paths:
        return None

    module_path = valid_paths[0]
    spec = importlib.util.spec_from_file_location(plugin_name, module_path)
    if spec.loader is None:
        raise errors.PluginError(f"unknown plugin: {plugin_name!r}")

    # Prevent mypy type complaints by asserting type.
    assert isinstance(spec.loader, importlib.abc.Loader)

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def _get_local_plugin_class(*, plugin_name: str, local_plugins_dir: Path):
    module = _load_local(plugin_name, local_plugins_dir)
    if not module:
        return
    logger.info(f"Loaded local plugin for {plugin_name}")

    # v2 requires plugin implementation to be named "PluginImpl".
    if hasattr(module, "PluginImpl") and issubclass(
        module.PluginImpl, plugins.v2.PluginV2
    ):
        return module.PluginImpl

    raise errors.PluginError(f"unknown plugin: {plugin_name!r}")


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


def _make_options(
    part_name, part_schema, definitions_schema, properties, plugin_schema
):
    # Make copies as these dictionaries are tampered with
    part_schema = part_schema.copy()
    properties = properties.copy()

    plugin_schema = _merged_part_and_plugin_schemas(
        part_schema, definitions_schema, plugin_schema
    )

    try:
        jsonschema.validate(properties, plugin_schema)
    except jsonschema.ValidationError as e:
        error = snapcraft_legacy.yaml_utils.errors.YamlValidationError.from_validation_error(
            e
        )
        raise errors.PluginError(
            "properties failed to load for {}: {}".format(part_name, error.message)
        )

    return _populate_options(properties, plugin_schema)


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
