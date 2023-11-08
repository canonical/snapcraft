# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, TextIO

import yaml
import yaml.error

from snapcraft import errors, utils
from snapcraft.extensions import apply_extensions
from snapcraft.projects import Architecture, GrammarAwareProject

from . import grammar
from snapcraft.const import ESM_BASES, LEGACY_BASES

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


def _check_duplicate_keys(node):
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


def _dict_constructor(loader, node):
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


class _SafeLoader(yaml.SafeLoader):  # pylint: disable=too-many-ancestors
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.add_constructor(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _dict_constructor
        )


def safe_load(filestream: TextIO) -> Dict[str, Any]:
    """Safe load and parse YAML-formatted file to a dictionary.

    :returns: A dictionary containing the yaml data.

    :raises SnapcraftError: if the file could not be loaded and parsed.
    """
    try:
        return yaml.safe_load(filestream)
    except yaml.error.YAMLError as err:
        raise errors.SnapcraftError(f"snapcraft.yaml parsing error: {err!s}") from err


def get_base(filestream: TextIO) -> Optional[str]:
    """Get the effective base from a snapcraft.yaml file.

    :param filename: The YAML file to load.

    :returns: Effective base of the project or None if the base cannot be determined.

    :raises SnapcraftError: If the yaml could not be loaded.
    """
    data = safe_load(filestream)
    return utils.get_effective_base(
        base=data.get("base"),
        build_base=data.get("build-base"),
        project_type=data.get("type"),
        name=data.get("name"),
    )


def load(filestream: TextIO) -> Dict[str, Any]:
    """Load and parse a YAML-formatted file.

    :param filename: The YAML file to load.

    :returns: A dictionary of the yaml data.

    :raises SnapcraftError: if loading didn't succeed.
    :raises LegacyFallback: if the project's base is a legacy base.
    :raises MaintenanceBase: if the base is not supported.
    """
    build_base = get_base(filestream)

    if build_base is None:
        raise errors.LegacyFallback("no base defined")
    if build_base in ESM_BASES:
        raise errors.MaintenanceBase(build_base)
    if build_base in LEGACY_BASES:
        raise errors.LegacyFallback(f"base is {build_base}")

    filestream.seek(0)

    try:
        return yaml.load(
            filestream,
            Loader=_SafeLoader,  # noqa: S506 Probable unsafe use of yaml.load()
        )
    except yaml.error.YAMLError as err:
        raise errors.SnapcraftError(f"snapcraft.yaml parsing error: {err!s}") from err


def apply_yaml(
    yaml_data: Dict[str, Any], build_on: str, build_for: str
) -> Dict[str, Any]:
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

    yaml_data = apply_extensions(yaml_data, arch=build_on, target_arch=build_for)

    if "parts" in yaml_data:
        yaml_data["parts"] = grammar.process_parts(
            parts_yaml_data=yaml_data["parts"], arch=build_on, target_arch=build_for
        )

    # replace all architectures with the architectures in the current build plan
    yaml_data["architectures"] = [Architecture(build_on=build_on, build_for=build_for)]

    return yaml_data


def get_snap_project() -> _SnapProject:
    """Find the snapcraft.yaml to load.

    :raises SnapcraftError: if the project yaml file cannot be found.
    """
    for snap_project in _SNAP_PROJECT_FILES:
        if snap_project.project_file.exists():
            return snap_project

    raise errors.ProjectMissing()


def extract_parse_info(yaml_data: Dict[str, Any]) -> Dict[str, List[str]]:
    """Remove parse-info data from parts.

    :param yaml_data: The project YAML data.

    :return: The extracted parse info for each part.
    """
    parse_info: Dict[str, List[str]] = {}

    if "parts" in yaml_data:
        for name, data in yaml_data["parts"].items():
            if "parse-info" in data:
                parse_info[name] = data.pop("parse-info")

    return parse_info


def process_yaml(project_file: Path) -> Dict[str, Any]:
    """Process yaml data from file into a dictionary.

    :param project_file: Path to project.

    :raises SnapcraftError: if the project yaml file cannot be loaded.

    :return: The processed YAML data.
    """
    try:
        with open(project_file, encoding="utf-8") as yaml_file:
            yaml_data = load(yaml_file)
    except OSError as err:
        msg = err.strerror
        if err.filename:
            msg = f"{msg}: {err.filename!r}."
        raise errors.SnapcraftError(msg) from err

    return yaml_data
