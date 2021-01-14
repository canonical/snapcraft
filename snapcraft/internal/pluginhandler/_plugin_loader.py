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

import snapcraft.yaml_utils.errors
from snapcraft import plugins
from snapcraft.internal import errors
from snapcraft.project import Project

logger = logging.getLogger(__name__)


def load_plugin(
    plugin_name: str,
    part_name: str,
    project: Project,
    properties,
    part_schema,
    definitions_schema,
) -> plugins.v1.PluginV1:
    local_plugins_dir = project._get_local_plugins_dir()
    if local_plugins_dir is not None:
        plugin_class = _get_local_plugin_class(
            plugin_name=plugin_name, local_plugins_dir=local_plugins_dir
        )
    if plugin_class is None:
        plugin_class = plugins.get_plugin_for_base(
            plugin_name, build_base=project._get_build_base()
        )

    if issubclass(plugin_class, plugins.v2.PluginV2):
        plugin_schema = plugin_class.get_schema()
        options = _make_options(
            part_name, part_schema, definitions_schema, properties, plugin_schema
        )
        plugin = plugin_class(part_name=part_name, options=options)
    else:
        plugin_schema = plugin_class.schema()
        _validate_pull_and_build_properties(
            plugin_name, plugin_class, part_schema, definitions_schema
        )
        options = _make_options(
            part_name, part_schema, definitions_schema, properties, plugin_schema
        )
        plugin = plugin_class(part_name, options, project)

        if project.is_cross_compiling:
            logger.debug(
                "Setting {!r} as the compilation target for {!r}".format(
                    project.deb_arch, plugin_name
                )
            )
            plugin.enable_cross_compilation()

    return plugin


def _load_compat_x_prefix(plugin_name: str, module_name: str, local_plugin_dir: str):
    compat_path = Path(local_plugin_dir, f"x-{plugin_name}.py")
    if not compat_path.exists():
        return None

    preferred_name = f"{module_name}.py"
    logger.warning(
        f"Legacy plugin name detected, please rename the plugin's file name {compat_path.name!r} to {preferred_name!r}."
    )

    spec = importlib.util.spec_from_file_location(plugin_name, compat_path)
    if spec.loader is None:
        return None

    # Prevent mypy type complaints by asserting type.
    assert isinstance(spec.loader, importlib.abc.Loader)

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_local(plugin_name: str, local_plugin_dir: str):
    module_name = plugin_name.replace("-", "_")

    module = _load_compat_x_prefix(plugin_name, module_name, local_plugin_dir)
    if module is None:
        sys.path = [local_plugin_dir] + sys.path
        logger.debug(
            f"Loading plugin module {module_name!r} with sys.path {sys.path!r}"
        )
        try:
            module = importlib.import_module(module_name)
        finally:
            sys.path.pop(0)

    return module


def _get_local_plugin_class(*, plugin_name: str, local_plugins_dir: str):
    with contextlib.suppress(ImportError):
        module = _load_local(plugin_name, local_plugins_dir)
        logger.info(f"Loaded local plugin for {plugin_name}")

        # v2 requires plugin implementation to be named "PluginImpl".
        if hasattr(module, "PluginImpl") and issubclass(
            module.PluginImpl, plugins.v2.PluginV2
        ):
            return module.PluginImpl

        for attr in vars(module).values():
            if not isinstance(attr, type):
                continue
            if not issubclass(attr, plugins.v1.PluginV1):
                continue
            if not hasattr(attr, "__module__"):
                continue
            logger.debug(
                f"Plugin attribute {attr!r} has __module__: {attr.__module__!r}"
            )
            if attr.__module__.startswith("snapcraft.plugins"):
                continue
            return attr
        else:
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
        error = snapcraft.yaml_utils.errors.YamlValidationError.from_validation_error(e)
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
