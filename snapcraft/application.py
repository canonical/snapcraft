# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023-2024 Canonical Ltd.
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

import logging
import os
import sys
from typing import TYPE_CHECKING, Any

import craft_application.errors
import craft_cli
import craft_parts
import craft_store
from craft_application import Application, AppMetadata, launchpad, remote, util
from craft_application.commands import get_other_command_group
from craft_cli import emit
from craft_parts.plugins.dotnet_v2_plugin import DotnetV2Plugin
from overrides import override

import snapcraft
import snapcraft_legacy
from snapcraft import cli, commands, errors, models, services, store
from snapcraft.utils import get_effective_base
from snapcraft_legacy.cli import legacy

from .legacy_cli import _LIB_NAMES, _ORIGINAL_LIB_NAME_LOG_LEVEL
from .parts import plugins
from .parts.yaml_utils import get_snap_project

if TYPE_CHECKING:
    from craft_parts.plugins.plugins import PluginType

APP_METADATA = AppMetadata(
    name="snapcraft",
    summary="Package, distribute, and update snaps for Linux and IoT",
    ProjectClass=models.Project,
    source_ignore_patterns=["*.snap"],
    mandatory_adoptable_fields=list(models.MANDATORY_ADOPTABLE_FIELDS),
    docs_url="https://documentation.ubuntu.com/snapcraft/{version}",
)


MAPPED_ENV_VARS = {
    ev: "SNAP" + ev
    for ev in ("CRAFT_BUILD_FOR", "CRAFT_BUILD_ENVIRONMENT", "CRAFT_VERBOSITY_LEVEL")
}


def _get_esm_error_for_base(base: str) -> None:
    """Raise an error appropriate for the base under ESM."""
    match base:
        case "core":
            channel = "4.x"
            version = "4"
        case "core18":
            channel = "7.x"
            version = "7"
        case _:
            return

    raise errors.SnapcraftError(
        message=f"Base {base!r} is not supported by this version of Snapcraft.",
        resolution=(
            f"Use Snapcraft {version} from the {channel!r} channel of snapcraft where "
            f"{base!r} was last supported."
        ),
        doc_slug="/reference/bases",
    )


class Snapcraft(Application):
    """Snapcraft application definition."""

    _use_craftapp_lib: bool
    """True if the project should use the core24/craft-application codepath."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        # Locate the project file. It's used in early execution to determine
        # compatibility with previous versions of the snapcraft codebase, and in
        # the package service to copy the project file into the snap payload if
        # manifest generation is enabled.
        self._use_craftapp_lib = self._should_use_craftapp_lib()

        for craft_var, snapcraft_var in MAPPED_ENV_VARS.items():
            if env_val := os.getenv(snapcraft_var):
                os.environ[craft_var] = env_val

    def _should_use_craftapp_lib(self) -> bool:
        """Return true if the project is known to use Craft Application to build."""
        try:
            snapcraft_yaml_path = get_snap_project(self.project_dir).project_file
            with snapcraft_yaml_path.open() as file:
                _snapcraft_yaml_data = util.safe_yaml_load(file)
        # defer to the project service to raise errors
        except (
            craft_application.errors.ProjectFileError,
            craft_application.errors.YamlError,
        ):
            return False

        # When snapcraft.yaml exists but is empty
        if not isinstance(_snapcraft_yaml_data, dict):
            return False

        base = _snapcraft_yaml_data.get("base")
        build_base = _snapcraft_yaml_data.get("build-base")

        # Check for bases known *not* to use craft-application
        return all(
            non_craftapp_base not in (base, build_base)
            for non_craftapp_base in ("core18", "core20", "core22")
        )

    def _get_app_plugins(self) -> dict[str, PluginType]:
        return plugins.get_plugins(core22=False)

    @override
    def _register_default_plugins(self) -> None:
        """Register per application plugins when initializing."""
        super()._register_default_plugins()

        craft_parts.plugins.unregister("maven-use")

        if self._use_craftapp_lib:
            # core22 uses dotnet v1
            # core24 and newer uses dotnet v2
            craft_parts.plugins.unregister("dotnet")
            craft_parts.plugins.register({"dotnet": DotnetV2Plugin})

    @property
    def app_config(self) -> dict[str, Any]:
        """Overridden to add "core" knowledge to the config."""
        config = super().app_config
        config["use_craftapp_lib"] = self._use_craftapp_lib
        return config

    @override
    def _pre_run(self, dispatcher: craft_cli.Dispatcher) -> None:
        """Do any final setup before running the command.

        :raises SnapcraftError: If the wrong codebase is chosen for the project.
        :raises SnapcraftError: If the project uses a base that is not supported by the
          current version of Snapcraft.
        """
        if project := self._get_project_raw():
            # if the project metadata is incomplete, assume core24 so craft application
            # can present user-friendly errors when unmarshalling the model
            effective_base = (
                get_effective_base(
                    base=project.get("base"),
                    build_base=project.get("build-base"),
                    project_type=project.get("type"),
                    name=project.get("name"),
                )
                or "core24"
            )
            _get_esm_error_for_base(effective_base)

        super()._pre_run(dispatcher)

    @override
    def _run_inner(self) -> int:
        try:
            return_code = super()._run_inner()
        except craft_store.errors.NoKeyringError as err:
            self._emit_error(
                craft_cli.errors.CraftError(
                    f"{err}",
                    resolution=(
                        "Ensure the keyring is working or "
                        f"{store.constants.ENVIRONMENT_STORE_CREDENTIALS} "
                        "is correctly exported into the environment"
                    ),
                    docs_url="https://documentation.ubuntu.com/snapcraft/stable/how-to/publishing/authenticate",
                )
            )
            return_code = 1
        except craft_store.errors.CraftStoreError as err:
            self._emit_error(
                craft_cli.errors.CraftError(f"{err}", resolution=err.resolution),
                cause=err,
            )
            return_code = 1
        except remote.RemoteBuildError as err:
            err.doc_slug = "/explanation/remote-build"
            self._emit_error(err)
            return_code = err.retcode
        except launchpad.LaunchpadError as err:
            self._emit_error(
                craft_cli.errors.CraftError(
                    f"{err}", doc_slug="/explanation/remote-build"
                ),
                cause=err,
            )
            return_code = 1

        return return_code

    @override
    def _enable_craft_parts_features(self) -> None:
        """Enable partitions if components are defined."""
        try:
            project = self._get_project_raw()
        # defer to the project service to raise errors
        except (
            craft_application.errors.ProjectFileError,
            craft_application.errors.YamlError,
        ):
            return

        if project and project.get("components"):
            craft_parts.Features(enable_partitions=True)

    @staticmethod
    def _get_argv_command() -> str | None:
        """Return the first command name used as an argument."""
        command_names = {
            command.name
            for group in [
                *cli.COMMAND_GROUPS,
                cli.CORE22_LIFECYCLE_COMMAND_GROUP,
                cli.CORE24_LIFECYCLE_COMMAND_GROUP,
                get_other_command_group(),
            ]
            for command in group.commands
        }

        return next((arg for arg in sys.argv[1:] if arg in command_names), None)

    def _check_for_classic_fallback(self) -> None:
        """Check for and raise a ClassicFallback if an older codebase should be used.

        The project should use the classic fallback path for any of the following conditions.

        core20:
          1. Running a lifecycle command for a core20 snap
          2. Expanding extensions for a core20 snap
          3. Listing plugins for a core20 snap via the project metadata
          4. Listing plugins for a core20 snap via `snapcraft plugins --base core20`

        core22:
          5. Running a lifecycle command for a core22 snap

        Exception: If `--version` or `-V` is passed, do not use the classic fallback.

        If none of the above conditions are met, then the default craft-application
        code path should be used.

        :raises ClassicFallback: If the project should use the classic fallback code path.
        """
        argv_command = self._get_argv_command()

        # Exception: If `--version` or `-V` is passed, do not use the classic fallback.
        if {"--version", "-V"}.intersection(sys.argv):
            return

        if project := self._get_project_raw():
            # if the project metadata is incomplete, assume core24 so craft application
            # can present user-friendly errors when unmarshalling the model
            effective_base = (
                get_effective_base(
                    base=project.get("base"),
                    build_base=project.get("build-base"),
                    project_type=project.get("type"),
                    name=project.get("name"),
                )
                or "core24"
            )

            classic_lifecycle_commands = [
                command.name for command in cli.CORE22_LIFECYCLE_COMMAND_GROUP.commands
            ]

            if effective_base == "core20":
                # 1. Running a lifecycle command for a core20 snap
                # 2. Expanding extensions for a core20 snap
                # 3. Listing plugins for a core20 snap via the project metadata
                if argv_command is None or argv_command in [
                    *classic_lifecycle_commands,
                    "expand-extensions",
                    "list-plugins",
                    "plugins",
                ]:
                    raise errors.ClassicFallback()

            if effective_base == "core22":
                # 5. Running a lifecycle command for a core22 snap
                if argv_command is None or argv_command in classic_lifecycle_commands:
                    raise errors.ClassicFallback()

        # 4. Listing plugins for a core20 snap via `snapcraft list-plugins --base core20`
        if argv_command in ["list-plugins", "plugins"] and {
            "--base=core20",
            "core20",
        }.intersection(sys.argv):
            raise errors.ClassicFallback()

    @override
    def _get_dispatcher(self) -> craft_cli.Dispatcher:
        """Handle multiplexing of Snapcraft "codebases" depending on the project's base.

        ClassicFallback errors must be raised before creating the Dispatcher.

        - The codebase for core24 and newer commands uses craft-application to
          create and manage the Dispatcher.
        - The codebase for core22 commands creates its own Dispatcher and handles
            errors and exit codes.
        - The codebase for core20 commands uses the legacy snapcraft codebase which
            handles logging, errors, and exit codes internally.

        :raises ClassicFallback: If the core20 or core22 codebases should be used.
        """
        self._check_for_classic_fallback()
        return super()._get_dispatcher()

    @override
    def _create_dispatcher(self) -> craft_cli.Dispatcher:
        """Overridden to set the default command to "pack"."""
        return craft_cli.Dispatcher(
            self.app.name,
            self.command_groups,
            summary=str(self.app.summary),
            extra_global_args=self._global_arguments,
            default_command=commands.PackCommand,
        )

    def _get_project_raw(self) -> dict[str, Any] | None:
        """Get raw project data from the project service."""
        try:
            return self.services.get("project").get_raw()
        except craft_application.errors.ProjectFileError:
            return None


def create_app() -> Snapcraft:
    """Create a Snapcraft application with the proper commands."""
    services.register_snapcraft_services()
    snapcraft_services = services.SnapcraftServiceFactory(app=APP_METADATA)

    app = Snapcraft(
        app=APP_METADATA,
        services=snapcraft_services,
    )

    for group in [cli.CORE24_LIFECYCLE_COMMAND_GROUP, *cli.COMMAND_GROUPS]:
        app.add_command_group(group.name, group.commands)

    return app


def get_app_info() -> tuple[craft_cli.Dispatcher, dict[str, Any]]:
    """Retrieve application info. Used by craft-cli's completion module."""
    app = create_app()
    dispatcher = app._create_dispatcher()

    return dispatcher, app.app_config


def main() -> int:
    """Run craft-application based snapcraft with classic fallback."""
    if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
        snapcraft.ProjectOptions = snapcraft_legacy.ProjectOptions  # type: ignore
        legacy.legacy_run()
        return 0  # never called in normal operation

    # set lib loggers to debug level so that all messages are sent to Emitter
    for lib_name in _LIB_NAMES:
        logger = logging.getLogger(lib_name)
        _ORIGINAL_LIB_NAME_LOG_LEVEL[lib_name] = logger.level
        logger.setLevel(logging.DEBUG)

    app = create_app()

    try:
        return app.run()
    except errors.ClassicFallback:
        emit.debug("Falling back from craft-application to snapcraft.")
        return cli.run()
