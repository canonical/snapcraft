# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Parts lifecycle preparation and execution."""

from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict

import yaml
import yaml.error
from craft_cli import emit

from snapcraft import errors
from snapcraft.parts import PartsLifecycle
from snapcraft.projects import Project

if TYPE_CHECKING:
    import argparse


_PROJECT_FILES = [
    Path("snapcraft.yaml"),
    Path("snap/snapcraft.yaml"),
    Path("build-aux/snap/snapcraft.yaml"),
    Path(".snapcraft.yaml"),
]


def run(step_name: str, parsed_args: "argparse.Namespace") -> None:
    """Run the parts lifecycle.

    :raises SnapcraftError: if the step name is invalid, or the project
        yaml file cannot be loaded.
    :raises LegacyFallback: if the project's base is not core22.
    """
    emit.trace(f"{step_name} parts, arguments: {parsed_args}")
    yaml_data = {}
    for project_file in _PROJECT_FILES:
        if project_file.is_file():
            yaml_data = _load_yaml(project_file)
            break
    else:
        raise errors.SnapcraftError(
            "Could not find snap/snapcraft.yaml. Are you sure you are in the "
            "right directory?",
            resolution="To start a new project, use `snapcraft init`",
        )

    # only execute the new codebase from core22 onwards
    if yaml_data.get("base") != "core22":
        raise errors.LegacyFallback("base is not core22")

    # TODO: apply extensions
    # yaml_data = apply_extensions(yaml_data)

    # TODO: process grammar
    # yaml_data = process_grammar(yaml_data)

    project = Project.unmarshal(yaml_data)

    _run_step(step_name, project, parsed_args)


def _run_step(
    step_name: str, project: Project, parsed_args: "argparse.Namespace",
) -> None:

    # TODO: check destructive and managed modes and run in provider
    _ = parsed_args

    work_dir = Path("work").absolute()

    lifecycle = PartsLifecycle(project.parts, work_dir=work_dir)
    lifecycle.run(step_name)


def _load_yaml(filename: Path) -> Dict[str, Any]:
    """Load and parse a YAML-formatted file.

    :param filename: The YAML file to load.

    :raises SnapcraftError: if loading didn't succeed.
    """
    try:
        with open(filename, encoding="utf-8") as yaml_file:
            return yaml.safe_load(yaml_file)
    except OSError as err:
        msg = err.strerror
        if err.filename:
            msg = f"{msg}: {err.filename!r}."
        raise errors.SnapcraftError(msg) from err
    except yaml.error.YAMLError as err:
        raise errors.SnapcraftError(f"YAML parsing error: {err!s}") from err
