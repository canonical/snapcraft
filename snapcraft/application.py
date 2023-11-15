# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Main Snapcraft Application."""

from __future__ import annotations

import dataclasses
import logging
import os
import pathlib
from typing import Any, cast

from craft_application import AppConfig, Application, AppMetadata, commands, util
from craft_cli import emit
from overrides import override

from snapcraft import cli, command_modifiers, errors, models, services, utils
from snapcraft.commands import unimplemented
from snapcraft.parts import yaml_utils


@dataclasses.dataclass()
class SnapcraftConfig(AppConfig):
    """Snapcraft-specific application configuration."""

    project_dir: pathlib.Path = pathlib.Path.cwd()
    assets_dir: pathlib.Path = pathlib.Path.cwd() / "snap"
    build_for: str = util.get_host_architecture()


APP_METADATA = AppMetadata(
    name="snapcraft",
    summary="Package, distribute, and update snaps for Linux and IoT",
    ProjectClass=models.Project,
    AppConfig=SnapcraftConfig,
    source_ignore_patterns=["*.snap"],
)


class Snapcraft(Application):
    """Snapcraft application definition."""

    project: models.Project
    config: SnapcraftConfig

    def _resolve_project_path(self, project_dir: pathlib.Path | None) -> pathlib.Path:
        """Find snapcraft.yaml."""
        if project_dir is None:
            project_dir = pathlib.Path.cwd()
        old_cwd = os.getcwd()
        try:
            os.chdir(project_dir)
            project_metadata = yaml_utils.get_snap_project()
            project_path = project_metadata.project_file.resolve()
            assets_dir = project_metadata.project_file.resolve()
        finally:
            os.chdir(old_cwd)

        self._assets_dir = assets_dir
        return project_path

    def _project_vars(self, yaml_data: dict[str, Any]) -> dict[str, str]:
        """Get project variables for craft_parts.ProjectInfo."""
        return {
            "version": cast(str, yaml_data.get("version", "")),
            "grade": cast(str, yaml_data.get("grade", "")),
        }

    @override
    def _extra_yaml_transform(self, yaml_data: dict[str, Any]) -> dict[str, Any]:
        """Snapcraft-specific transforms."""
        host_arch = utils.get_host_architecture()
        # Apply craft-grammar (this runs in the manager and in managed mode)
        yaml_data = yaml_utils.apply_yaml(yaml_data, host_arch, self.config.build_for)

        # TODO: Fill info from adopt-info, etc.

        return yaml_data

    @override
    def _configure_services(self, platform: str | None, build_for: str | None) -> None:
        if build_for is None:
            build_for = util.get_host_architecture()

        self.services.set_kwargs("image", work_dir=self._work_dir, build_for=build_for)
        self.services.set_kwargs(
            "package",
            platform=platform,
            build_for=build_for,
        )
        super()._configure_services(platform, build_for)

    @property
    def command_groups(self):
        """Short-circuit the standard command groups for now."""
        # TODO: Remove this once we've got lifecycle commands and version migrated.
        return self._command_groups

    def run(self) -> None:
        """Fall back to the old snapcraft entrypoint."""
        self._get_dispatcher()
        raise errors.ClassicFallback()


def main() -> int:
    """Run craft-application based charmcraft with classic fallback."""
    for lib_name in (
        "craft_parts",
        "craft_providers",
        "craft_store",
        "snapcraft.remote",
    ):
        logger = logging.getLogger(lib_name)
        logger.setLevel(logging.DEBUG)

    charmcraft_services = services.SnapcraftServiceFactory(app=APP_METADATA)

    commands.lifecycle.LifecyclePartsCommand.register_parser_filler(
        command_modifiers.fill_lifecycle_parser
    )
    commands.lifecycle.LifecyclePartsCommand.register_prologue(
        command_modifiers.lifecycle_prologue
    )

    app = Snapcraft(app=APP_METADATA, services=charmcraft_services)

    app.add_command_group(
        "Lifecycle",
        [
            unimplemented.Clean,
            unimplemented.Pull,
            unimplemented.Build,
            unimplemented.Stage,
            unimplemented.Prime,
            unimplemented.Pack,
            unimplemented.RemoteBuild,
            unimplemented.Snap,  # Hidden (legacy compatibility)
            unimplemented.Plugins,
            unimplemented.ListPlugins,
            unimplemented.Try,
        ],
    )
    app.add_command_group(
        "Extensions",
        [
            unimplemented.ListExtensions,
            unimplemented.Extensions,
            unimplemented.ExpandExtensions,
        ],
    )
    app.add_command_group(
        "Store Account",
        [
            unimplemented.Login,
            unimplemented.ExportLogin,
            unimplemented.Logout,
            unimplemented.Whoami,
        ],
    )
    app.add_command_group(
        "Store Snap Names",
        [
            unimplemented.Register,
            unimplemented.Names,
            unimplemented.ListRegistered,
            unimplemented.List,
            unimplemented.Metrics,
            unimplemented.UploadMetadata,
        ],
    )
    app.add_command_group(
        "Store Snap Release Management",
        [
            unimplemented.Release,
            unimplemented.Close,
            unimplemented.Status,
            unimplemented.Upload,
            unimplemented.Push,
            unimplemented.Promote,
            unimplemented.ListRevisions,
            unimplemented.Revisions,
        ],
    )
    app.add_command_group(
        "Store Snap Tracks",
        [
            unimplemented.ListTracks,
            unimplemented.Tracks,
            unimplemented.SetDefaultTrack,
        ],
    )
    app.add_command_group(
        "Store Key Management",
        [
            unimplemented.CreateKey,
            unimplemented.RegisterKey,
            unimplemented.SignBuild,
            unimplemented.ListKeys,
        ],
    )
    app.add_command_group(
        "Store Validation Sets",
        [
            unimplemented.EditValidationSets,
            unimplemented.ListValidationSets,
            unimplemented.Validate,
            unimplemented.Gated,
        ],
    )
    app.add_command_group(
        "Other",
        [
            unimplemented.Version,
            unimplemented.Lint,
            unimplemented.Init,
        ],
    )

    try:
        return app.run()
    except errors.ClassicFallback:
        emit.debug("Falling back to Snapcraft Classic™")
        return cli.run()
