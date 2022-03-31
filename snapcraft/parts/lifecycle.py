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

import subprocess
from pathlib import Path
from typing import TYPE_CHECKING, cast

from craft_cli import emit
from craft_parts import infos

from snapcraft import errors, extensions, pack, providers, utils
from snapcraft.meta import snap_yaml
from snapcraft.parts import PartsLifecycle
from snapcraft.projects import GrammarAwareProject, Project
from snapcraft.providers import capture_logs_from_instance

from . import grammar, yaml_utils

if TYPE_CHECKING:
    import argparse


_PROJECT_FILES = [
    Path("snapcraft.yaml"),
    Path("snap/snapcraft.yaml"),
    Path("build-aux/snap/snapcraft.yaml"),
    Path(".snapcraft.yaml"),
]


def run(command_name: str, parsed_args: "argparse.Namespace") -> None:
    """Run the parts lifecycle.

    :raises SnapcraftError: if the step name is invalid, or the project
        yaml file cannot be loaded.
    :raises LegacyFallback: if the project's base is not core22.
    """
    emit.trace(f"command: {command_name}, arguments: {parsed_args}")
    yaml_data = {}
    assets_dir = Path("snap")

    for project_file in _PROJECT_FILES:
        if project_file.is_file():

            if project_file.parent.name == "snap":
                assets_dir = project_file.parent

            try:
                with open(project_file, encoding="utf-8") as yaml_file:
                    yaml_data = yaml_utils.load(yaml_file)
                break
            except OSError as err:
                msg = err.strerror
                if err.filename:
                    msg = f"{msg}: {err.filename!r}."
                raise errors.SnapcraftError(msg) from err
    else:
        raise errors.SnapcraftError(
            "Could not find snap/snapcraft.yaml. Are you sure you are in the "
            "right directory?",
            resolution="To start a new project, use `snapcraft init`",
        )

    # validate project grammar
    GrammarAwareProject.validate_grammar(yaml_data)

    # argument --provider is only supported by legacy snapcraft
    if parsed_args.provider:
        raise errors.SnapcraftError("Option --provider is not supported.")

    # TODO: support for target_arch
    arch = _get_arch()
    yaml_data = extensions.apply_extensions(yaml_data, arch=arch, target_arch=arch)

    if "parts" in yaml_data:
        yaml_data["parts"] = grammar.process_parts(
            parts_yaml_data=yaml_data["parts"], arch=arch, target_arch=arch
        )

    project = Project.unmarshal(yaml_data)

    _run_command(
        command_name, project=project, assets_dir=assets_dir, parsed_args=parsed_args
    )


def _run_command(
    command_name: str,
    *,
    project: Project,
    assets_dir: Path,
    parsed_args: "argparse.Namespace",
) -> None:
    managed_mode = utils.is_managed_mode()

    if not managed_mode and not parsed_args.destructive_mode:
        _run_in_provider(project, command_name, parsed_args)
        return

    if managed_mode:
        work_dir = utils.get_managed_environment_home_path()
    else:
        work_dir = Path.cwd()

    step_name = "prime" if command_name == "pack" else command_name

    lifecycle = PartsLifecycle(
        project.parts,
        work_dir=work_dir,
        assets_dir=assets_dir,
        package_repositories=project.package_repositories,
    )
    lifecycle.run(step_name)

    snap_yaml.write(project, lifecycle.prime_dir, arch=lifecycle.target_arch)

    if command_name == "pack":
        pack.pack_snap(
            lifecycle.prime_dir,
            output=parsed_args.output,
            compression=project.compression,
        )


def _run_in_provider(project: Project, command_name: str, parsed_args: "argparse.Namespace"):
    """Pack image in provider instance."""
    emit.trace("Checking build provider availability")
    provider_name = "lxd" if parsed_args.use_lxd else None
    provider = providers.get_provider(provider_name)
    provider.ensure_provider_is_available()

    cmd = ["snapcraft", command_name]

    if hasattr(parsed_args, "parts"):
        cmd.extend(parsed_args.parts)

    output_dir = utils.get_managed_environment_project_path()

    # FIXME: pause emitter when executing instance (needs craft-cli support)
    emit.progress("Launching instance...")
    with provider.launched_environment(
        project_name=project.name, project_path=Path().absolute(), base=cast(str, project.base)
    ) as instance:
        try:
            emit.message("Launched instance", intermediate=True)
            instance.execute_run(cmd, check=True, cwd=output_dir)
        except subprocess.CalledProcessError as err:
            capture_logs_from_instance(instance)
            raise providers.ProviderError(
                f"Failed to pack image '{project.name}:{project.version}'."
            ) from err


# TODO Needs exposure from craft-parts.
def _get_arch() -> str:
    machine = infos._get_host_architecture()  # pylint: disable=protected-access
    # FIXME Raise the potential KeyError.
    return infos._ARCH_TRANSLATIONS[machine]["deb"]  # pylint: disable=protected-access
