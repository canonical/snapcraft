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

import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, cast

from craft_cli import EmitterMode, emit
from craft_parts import infos

from snapcraft import errors, extensions, pack, providers, utils
from snapcraft.meta import snap_yaml
from snapcraft.parts import PartsLifecycle
from snapcraft.projects import GrammarAwareProject, Project
from snapcraft.providers import capture_logs_from_instance

from . import grammar, yaml_utils
from .setup_assets import setup_assets
from .update_metadata import update_project_metadata

if TYPE_CHECKING:
    import argparse


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


def get_snap_project() -> _SnapProject:
    """Find the snapcraft.yaml to load.

    :raises SnapcraftError: if the project yaml file cannot be found.
    """
    for snap_project in _SNAP_PROJECT_FILES:
        if snap_project.project_file.exists():
            return snap_project

    raise errors.SnapcraftError(
        "Could not find snap/snapcraft.yaml. Are you sure you are in the "
        "right directory?",
        resolution="To start a new project, use `snapcraft init`",
    )


def process_yaml(project_file: Path) -> Dict[str, Any]:
    """Process the yaml from project file.

    :raises SnapcraftError: if the project yaml file cannot be loaded.
    """
    yaml_data = {}

    try:
        with open(project_file, encoding="utf-8") as yaml_file:
            yaml_data = yaml_utils.load(yaml_file)
    except OSError as err:
        msg = err.strerror
        if err.filename:
            msg = f"{msg}: {err.filename!r}."
        raise errors.SnapcraftError(msg) from err

    # validate project grammar
    GrammarAwareProject.validate_grammar(yaml_data)

    # TODO: support for target_arch
    arch = _get_arch()
    yaml_data = extensions.apply_extensions(yaml_data, arch=arch, target_arch=arch)

    if "parts" in yaml_data:
        yaml_data["parts"] = grammar.process_parts(
            parts_yaml_data=yaml_data["parts"], arch=arch, target_arch=arch
        )

    return yaml_data


def _extract_parse_info(yaml_data: Dict[str, Any]) -> Dict[str, List[str]]:
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


def run(command_name: str, parsed_args: "argparse.Namespace") -> None:
    """Run the parts lifecycle.

    :raises SnapcraftError: if the step name is invalid, or the project
        yaml file cannot be loaded.
    :raises LegacyFallback: if the project's base is not core22.
    """
    emit.trace(f"command: {command_name}, arguments: {parsed_args}")

    snap_project = get_snap_project()
    yaml_data = process_yaml(snap_project.project_file)
    parse_info = _extract_parse_info(yaml_data)

    if parsed_args.provider:
        raise errors.SnapcraftError("Option --provider is not supported.")

    project = Project.unmarshal(yaml_data)

    _run_command(
        command_name,
        project=project,
        parse_info=parse_info,
        assets_dir=snap_project.assets_dir,
        parsed_args=parsed_args,
    )


def _run_command(
    command_name: str,
    *,
    project: Project,
    parse_info: Dict[str, List[str]],
    assets_dir: Path,
    parsed_args: "argparse.Namespace",
) -> None:
    managed_mode = utils.is_managed_mode()
    part_names = getattr(parsed_args, "parts", None)

    if not managed_mode and command_name == "snap":
        emit.message(
            "The 'snap' command is deprecated, use 'pack' instead.", intermediate=True
        )

    if parsed_args.use_lxd and providers.get_platform_default_provider() == "lxd":
        emit.message("LXD is used by default on this platform.", intermediate=True)

    if (
        not managed_mode
        and not parsed_args.destructive_mode
        and not os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "host"
    ):
        if command_name == "clean" and not part_names:
            _clean_provider(project, parsed_args)
        else:
            _run_in_provider(project, command_name, parsed_args)
        return

    if managed_mode:
        work_dir = utils.get_managed_environment_home_path()
    else:
        work_dir = Path.cwd()

    step_name = "prime" if command_name in ("pack", "snap") else command_name

    lifecycle = PartsLifecycle(
        project.parts,
        work_dir=work_dir,
        assets_dir=assets_dir,
        base=project.base,
        package_repositories=project.package_repositories,
        part_names=part_names,
        adopt_info=project.adopt_info,
        project_name=project.name,
        parse_info=parse_info,
        project_vars={
            "version": project.version or "",
            "grade": project.grade or "",
        },
    )
    if command_name == "clean":
        lifecycle.clean(part_names=part_names)
        return

    lifecycle.run(step_name)

    # Extract metadata and generate snap.yaml
    project_vars = lifecycle.project_vars
    if step_name == "prime" and not part_names:
        emit.progress("Extracting and updating metadata...")
        metadata_list = lifecycle.extract_metadata()
        update_project_metadata(
            project,
            project_vars=project_vars,
            metadata_list=metadata_list,
            assets_dir=assets_dir,
            prime_dir=lifecycle.prime_dir,
        )

        emit.progress("Copying snap assets...")
        setup_assets(
            project,
            assets_dir=assets_dir,
            prime_dir=lifecycle.prime_dir,
        )

        emit.progress("Generating snap metadata...")
        snap_yaml.write(
            project,
            lifecycle.prime_dir,
            arch=lifecycle.target_arch,
        )
        emit.message("Generated snap metadata", intermediate=True)

    if command_name in ("pack", "snap"):
        pack.pack_snap(
            lifecycle.prime_dir,
            output=parsed_args.output,
            compression=project.compression,
        )


def _clean_provider(project: Project, parsed_args: "argparse.Namespace") -> None:
    """Clean the provider environment.

    :param project: The project to clean.
    """
    emit.trace("Clean build provider")
    provider_name = "lxd" if parsed_args.use_lxd else None
    provider = providers.get_provider(provider_name)
    instance_names = provider.clean_project_environments(
        project_name=project.name, project_path=Path().absolute()
    )
    if instance_names:
        emit.message(f"Removed instance: {', '.join(instance_names)}")
    else:
        emit.message("No instances to remove")


def _run_in_provider(
    project: Project, command_name: str, parsed_args: "argparse.Namespace"
) -> None:
    """Pack image in provider instance."""
    emit.trace("Checking build provider availability")
    provider_name = "lxd" if parsed_args.use_lxd else None
    provider = providers.get_provider(provider_name)
    provider.ensure_provider_is_available()

    cmd = ["snapcraft", command_name]

    if hasattr(parsed_args, "parts"):
        cmd.extend(parsed_args.parts)

    if getattr(parsed_args, "output", None):
        cmd.extend(["--output", parsed_args.output])

    if emit.get_mode() == EmitterMode.VERBOSE:
        cmd.append("--verbose")
    elif emit.get_mode() == EmitterMode.QUIET:
        cmd.append("--quiet")
    elif emit.get_mode() == EmitterMode.TRACE:
        cmd.append("--trace")

    output_dir = utils.get_managed_environment_project_path()

    emit.progress("Launching instance...")
    with provider.launched_environment(
        project_name=project.name,
        project_path=Path().absolute(),
        base=cast(str, project.base),
    ) as instance:
        try:
            with emit.pause():
                instance.execute_run(cmd, check=True, cwd=output_dir)
            capture_logs_from_instance(instance)
        except subprocess.CalledProcessError as err:
            capture_logs_from_instance(instance)
            raise providers.ProviderError(
                f"Failed to execute {command_name} in instance."
            ) from err


# TODO Needs exposure from craft-parts.
def _get_arch() -> str:
    machine = infos._get_host_architecture()  # pylint: disable=protected-access
    # FIXME Raise the potential KeyError.
    return infos._ARCH_TRANSLATIONS[machine]["deb"]  # pylint: disable=protected-access
