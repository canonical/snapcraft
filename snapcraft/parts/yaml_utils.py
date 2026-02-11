# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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

import os
from collections.abc import Hashable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO, cast

import craft_application.errors
import craft_cli
import yaml
import yaml.error
from craft_parts import ProjectVar, ProjectVarInfo

from snapcraft import const, errors, utils
from snapcraft.extensions import apply_extensions
from snapcraft.models import GrammarAwareProject

from . import grammar

_CORE_PART_KEYS = ["build-packages", "build-snaps"]
_CORE_PART_NAME = "snapcraft/core"


@dataclass
class _SnapProject:
    project_file: Path
    assets_dir: Path = Path("snap")


_SNAP_PROJECT_FILES = [
    _SnapProject(project_file=Path("snapcraft.yaml")),
    _SnapProject(project_file=Path("snap/snapcraft.yaml")),
    _SnapProject(
        project_file=Path("build-aux/snap/snapcraft.yaml"),
        assets_dir=Path("build-aux/snap"),
    ),
    _SnapProject(project_file=Path(".snapcraft.yaml")),
]


def _check_duplicate_keys(node: yaml.Node) -> None:
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


def _dict_constructor(
    loader: yaml.Loader, node: yaml.MappingNode
) -> dict[Hashable, Any]:
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


class _SafeLoader(yaml.SafeLoader):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.add_constructor(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _dict_constructor
        )


def safe_load(filestream: TextIO) -> dict[str, Any]:
    """Safe load and parse YAML-formatted file to a dictionary.

    :returns: A dictionary containing the yaml data.

    :raises SnapcraftError: if the file could not be loaded and parsed.
    """
    try:
        return yaml.safe_load(filestream)
    except yaml.error.YAMLError as err:
        raise errors.SnapcraftError(f"snapcraft.yaml parsing error: {err!s}") from err


def get_base(filestream: TextIO) -> str | None:
    """Get the effective base from a snapcraft.yaml file.

    :param filename: The YAML file to load.

    :returns: Effective base of the project or None if the base cannot be determined.

    :raises SnapcraftError: If the yaml could not be loaded.
    """
    data = safe_load(filestream)
    return get_base_from_yaml(data)


def get_base_from_yaml(data: dict[str, Any]) -> str | None:
    """Get the effective base from a dictionary of yaml data.

    :param data: The YAML data to load.

    :returns: Effective base of the project or None if the base cannot be determined.

    :raises SnapcraftError: If the yaml could not be loaded.
    """
    return utils.get_effective_base(
        base=data.get("base"),
        build_base=data.get("build-base"),
        project_type=data.get("type"),
        name=data.get("name"),
    )


def load(filestream: TextIO) -> dict[str, Any]:
    """Load and parse a YAML-formatted file.

    :param filename: The YAML file to load.

    :returns: A dictionary of the yaml data.

    :raises SnapcraftError: if loading didn't succeed.
    :raises MissingBase: if there isn't a base in the project file.
    :raises MaintenanceBase: if the base is not supported.
    """
    build_base = get_base(filestream)

    if build_base is None:
        raise errors.MissingBase()
    if build_base in const.ESM_BASES:
        raise errors.MaintenanceBase(build_base)

    filestream.seek(0)

    try:
        return yaml.load(
            filestream,
            Loader=_SafeLoader,  # noqa: S506 Probable unsafe use of yaml.load()
        )
    except yaml.error.YAMLError as err:
        raise errors.SnapcraftError(f"snapcraft.yaml parsing error: {err!s}") from err


def apply_yaml(
    yaml_data: dict[str, Any], build_on: str, build_for: str
) -> dict[str, Any]:
    """Apply Snapcraft logic to yaml_data.

    Extensions are applied and advanced grammar is processed.
    The architectures data is reduced to architectures in the current build plan.

    :param yaml_data: The project YAML data.
    :param build_on: Architecture the snap project will be built on.
    :param build_for: Target architecture the snap project will be built to.

    :return: A dictionary of yaml data with snapcraft logic applied.
    """
    # validate project grammar
    GrammarAwareProject.validate_grammar(yaml_data)

    # Special Snapcraft Part
    core_part = {k: yaml_data.pop(k) for k in _CORE_PART_KEYS if k in yaml_data}
    if core_part:
        core_part["plugin"] = "nil"
        yaml_data["parts"][_CORE_PART_NAME] = core_part

    apply_extensions(yaml_data, arch=build_on, target_arch=build_for)

    if "parts" in yaml_data:
        yaml_data["parts"] = grammar.process_parts(
            parts_yaml_data=yaml_data["parts"], arch=build_on, target_arch=build_for
        )

    if any(
        b.startswith("core22")
        for b in (
            yaml_data.get("base", ""),
            yaml_data.get("build-base", ""),
            yaml_data.get("name", ""),
        )
    ):
        # replace all architectures with the architectures in the current build plan
        yaml_data["architectures"] = [{"build-on": build_on, "build-for": build_for}]
    else:
        # replace all platforms with the platform in the current build plan
        yaml_data["platforms"] = {
            build_for: {"build-on": build_on, "build-for": build_for}
        }

    return yaml_data


def get_snap_project(project_dir: Path | None = None) -> _SnapProject:
    """Find the snapcraft.yaml to load.

    :param project_dir: The directory to search for the project yaml file. If not
    provided, the current working directory is used.

    :raises ProjectDirectoryMissingError: if the project directory does not exist.
    :raises ProjectDirectoryTypeError: if the project directory is not a directory.
    :raises ProjectFileMissingError: if the project file is not found.
    """
    if project_dir:
        if not project_dir.exists():
            raise craft_application.errors.ProjectDirectoryMissingError(project_dir)
        if not project_dir.is_dir():
            raise craft_application.errors.ProjectDirectoryTypeError(project_dir)
    else:
        project_dir = Path.cwd()

    for snap_project in _SNAP_PROJECT_FILES:
        try:
            (project_dir / snap_project.project_file).resolve(strict=True)
        except FileNotFoundError:
            continue
        except NotADirectoryError:
            raise craft_application.errors.ProjectDirectoryTypeError(
                snap_project.project_file,
                details="The 'snap' name is reserved for the project directory.",
                resolution="Rename or remove the file named 'snap'.",
            )

        return snap_project

    raise craft_application.errors.ProjectFileMissingError(
        f"Project file 'snapcraft.yaml' not found in '{project_dir}'.",
        details="The project file could not be found.",
        resolution="Ensure the project file exists.",
        retcode=os.EX_NOINPUT,
    )


def extract_parse_info(yaml_data: dict[str, Any]) -> dict[str, list[str]]:
    """Remove parse-info data from parts.

    :param yaml_data: The project YAML data.

    :return: The extracted parse info for each part.
    """
    parse_info: dict[str, list[str]] = {}

    if "parts" in yaml_data:
        for name, data in yaml_data["parts"].items():
            if "parse-info" in data:
                parse_info[name] = data.pop("parse-info")

    return parse_info


def process_yaml(project_file: Path) -> dict[str, Any]:
    """Process yaml data from file into a dictionary.

    :param project_file: Path to project.

    :raises SnapcraftError: if the project yaml file cannot be loaded.
    :raises MissingBase: if there isn't a base in the project file.
    :raises MaintenanceBase: if the base is not supported.

    :return: The processed YAML data.
    """
    try:
        with open(project_file, encoding="utf-8") as yaml_file:
            yaml_data = load(yaml_file)
    except OSError as err:
        msg = err.strerror
        if err.filename:
            msg = f"{msg}: {err.filename!r}."
        # Casting as a str as OSError should always contain an error message
        raise errors.SnapcraftError(cast(str, msg)) from err

    return yaml_data


def create_project_vars(project: dict[str, Any]) -> ProjectVarInfo:
    """Create project variables for the version, grade, and each component's version.

    :param project: The project data.

    :returns: The project variables.
    """
    # always create project vars for the top-level version and grade
    project_vars: dict[str, Any] = {
        "version": ProjectVar(
            value=project.get("version"),
            part_name=project.get("adopt-info"),
        ).marshal(),
        "grade": ProjectVar(
            value=project.get("grade"),
            part_name=project.get("adopt-info"),
        ).marshal(),
    }

    # if components are defined, create a project var for each component's version
    # (which are different than the top-level version)
    if components := project.get("components"):
        project_vars["components"] = {}
        for name, component in components.items():
            project_vars["components"][name] = {}
            project_vars["components"][name]["version"] = ProjectVar(
                value=component.get("version"),
                part_name=component.get("adopt-info"),
            ).marshal()

    craft_cli.emit.debug(f"Created project variables {project_vars}.")
    return ProjectVarInfo.unmarshal(project_vars)
