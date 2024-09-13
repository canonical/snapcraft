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
import pathlib
import sys
from typing import Any

import craft_cli
import craft_parts
import craft_store
from craft_application import Application, AppMetadata, util
from craft_application.commands import get_other_command_group
from craft_cli import emit
from craft_parts.plugins.plugins import PluginType
from overrides import override

import snapcraft
import snapcraft_legacy
from snapcraft import cli, commands, errors, models, services, store
from snapcraft.extensions import apply_extensions
from snapcraft.models.project import SnapcraftBuildPlanner, apply_root_packages
from snapcraft.parts import set_global_environment
from snapcraft.utils import get_effective_base, get_host_architecture
from snapcraft_legacy.cli import legacy

from .legacy_cli import _LIB_NAMES, _ORIGINAL_LIB_NAME_LOG_LEVEL
from .parts import plugins
from .parts.yaml_utils import extract_parse_info

APP_METADATA = AppMetadata(
    name="snapcraft",
    summary="Package, distribute, and update snaps for Linux and IoT",
    ProjectClass=models.Project,
    BuildPlannerClass=SnapcraftBuildPlanner,
    source_ignore_patterns=["*.snap"],
    project_variables=["version", "grade"],
    mandatory_adoptable_fields=["version", "summary", "description"],
    docs_url="https://canonical-snapcraft.readthedocs-hosted.com/en/{version}",
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
    )


class Snapcraft(Application):
    """Snapcraft application definition."""

    _known_core24: bool
    """True if the project should use the core24/craft-application codepath."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._parse_info: dict[str, list[str]] = {}

        # Locate the project file. It's used in early execution to determine
        # compatibility with previous versions of the snapcraft codebase, and in
        # the package service to copy the project file into the snap payload if
        # manifest generation is enabled.
        try:
            self._snapcraft_yaml_path: pathlib.Path | None = self._resolve_project_path(
                None
            )
            with self._snapcraft_yaml_path.open() as file:
                self._snapcraft_yaml_data = util.safe_yaml_load(file)
        except FileNotFoundError:
            self._snapcraft_yaml_path = self._snapcraft_yaml_data = None

        self._known_core24 = self._get_known_core24()

        for craft_var, snapcraft_var in MAPPED_ENV_VARS.items():
            if env_val := os.getenv(snapcraft_var):
                os.environ[craft_var] = env_val

    def _get_known_core24(self) -> bool:
        """Return true if the project is known to be core24."""
        if self._snapcraft_yaml_data:
            base = self._snapcraft_yaml_data.get("base")
            build_base = self._snapcraft_yaml_data.get("build-base")

            # We know for sure that we're handling a core24 project
            if "core24" in (base, build_base) or build_base == "devel":
                return True

        return False

    def _get_app_plugins(self) -> dict[str, PluginType]:
        return plugins.get_plugins(core22=False)

    @override
    def _register_default_plugins(self) -> None:
        """Register per application plugins when initializing."""
        super()._register_default_plugins()

        # poetry plugin needs integration work, see #5025
        craft_parts.plugins.unregister("poetry")

        if self._known_core24:
            # dotnet is disabled for core24 and newer because it is pending a rewrite
            craft_parts.plugins.unregister("dotnet")

    @override
    def _configure_services(self, provider_name: str | None) -> None:
        self.services.set_kwargs(
            "package",
            build_plan=self._build_plan,
            snapcraft_yaml_path=self._snapcraft_yaml_path,
            parse_info=self._parse_info,
        )

        super()._configure_services(provider_name)

    @property
    def command_groups(self):
        """Replace craft-application's LifecycleCommand group."""
        _command_groups = super().command_groups
        for index, command_group in enumerate(_command_groups):
            if command_group.name == "Lifecycle":
                _command_groups[index] = cli.CORE24_LIFECYCLE_COMMAND_GROUP

        return _command_groups

    @override
    def _resolve_project_path(self, project_dir: pathlib.Path | None) -> pathlib.Path:
        """Overridden to handle the two possible locations for snapcraft.yaml."""
        if project_dir is None:
            project_dir = pathlib.Path.cwd()

        try:
            return super()._resolve_project_path(project_dir / "snap")
        except FileNotFoundError:
            try:
                return super()._resolve_project_path(project_dir)
            except FileNotFoundError:
                return super()._resolve_project_path(project_dir / "build-aux" / "snap")

    @property
    def app_config(self) -> dict[str, Any]:
        """Overridden to add "core" knowledge to the config."""
        config = super().app_config
        config["core24"] = self._known_core24
        return config

    @override
    def _pre_run(self, dispatcher: craft_cli.Dispatcher) -> None:
        """Do any final setup before running the command.

        :raises SnapcraftError: If the wrong codebase is chosen for the project.
        :raises SnapcraftError: If the project uses a base that is not supported by the
          current version of Snapcraft.
        """
        if self._snapcraft_yaml_data:
            # if the project metadata is incomplete, assume core24 so craft application
            # can present user-friendly errors when unmarshalling the model
            effective_base = (
                get_effective_base(
                    base=self._snapcraft_yaml_data.get("base"),
                    build_base=self._snapcraft_yaml_data.get("build-base"),
                    project_type=self._snapcraft_yaml_data.get("type"),
                    name=self._snapcraft_yaml_data.get("name"),
                )
                or "core24"
            )
            _get_esm_error_for_base(effective_base)
            self._ensure_remote_build_supported(effective_base)

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
                    docs_url="https://snapcraft.io/docs/snapcraft-authentication",
                )
            )
            return_code = 1
        except craft_store.errors.CraftStoreError as err:
            self._emit_error(
                craft_cli.errors.CraftError(f"{err}", resolution=err.resolution),
                cause=err,
            )
            return_code = 1

        return return_code

    @override
    def _enable_craft_parts_features(self) -> None:
        """Enable partitions if components are defined."""
        if self._snapcraft_yaml_data and self._snapcraft_yaml_data.get("components"):
            craft_parts.Features(enable_partitions=True)

    @override
    def _setup_partitions(self, yaml_data: dict[str, Any]) -> list[str] | None:
        components = models.ComponentProject.unmarshal(yaml_data)
        if components.components is None:
            return None

        return components.get_partitions()

    @override
    def _extra_yaml_transform(
        self, yaml_data: dict[str, Any], *, build_on: str, build_for: str | None
    ) -> dict[str, Any]:
        arch = build_on
        target_arch = build_for if build_for else get_host_architecture()
        new_yaml_data = apply_extensions(yaml_data, arch=arch, target_arch=target_arch)
        self._parse_info = extract_parse_info(new_yaml_data)
        return apply_root_packages(new_yaml_data)

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
          4. Remote builds for a core20 snap
          5. Listing plugins for a core20 snap via `snapcraft list-plugins --base core20`

        core22:
          6. Running a lifecycle command for a core22 snap
          7. Remote builds for a core22 snap with the `force-fallback` strategy

        Exception: If `--version` or `-V` is passed, do not use the classic fallback.

        If none of the above conditions are met, then the default craft-application
        code path should be used.

        :raises ClassicFallback: If the project should use the classic fallback code path.
        """
        argv_command = self._get_argv_command()
        build_strategy = os.environ.get("SNAPCRAFT_REMOTE_BUILD_STRATEGY", None)

        # Exception: If `--version` or `-V` is passed, do not use the classic fallback.
        if {"--version", "-V"}.intersection(sys.argv):
            return

        if self._snapcraft_yaml_data:
            # if the project metadata is incomplete, assume core24 so craft application
            # can present user-friendly errors when unmarshalling the model
            effective_base = (
                get_effective_base(
                    base=self._snapcraft_yaml_data.get("base"),
                    build_base=self._snapcraft_yaml_data.get("build-base"),
                    project_type=self._snapcraft_yaml_data.get("type"),
                    name=self._snapcraft_yaml_data.get("name"),
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

                # 4. Remote builds for a core20 snap
                # Note that a `core20` snap with 'disable-fallback' set will follow the
                # craft-application codepath and an error will be raised after the
                # dispatcher is created.
                if (
                    argv_command == "remote-build"
                    and not build_strategy
                    or build_strategy == "force-fallback"
                ):
                    raise errors.ClassicFallback()

            if effective_base == "core22":
                # 6. Running a lifecycle command for a core22 snap
                if argv_command is None or argv_command in classic_lifecycle_commands:
                    raise errors.ClassicFallback()

                # 7. Remote builds for a core22 snap with the `force-fallback` strategy
                if (
                    argv_command == "remote-build"
                    and build_strategy == "force-fallback"
                ):
                    raise errors.ClassicFallback()

        # 5. Listing plugins for a core20 snap via `snapcraft list-plugins --base core20`
        if argv_command in ["list-plugins", "plugins"] and {
            "--base=core20",
            "core20",
        }.intersection(sys.argv):
            raise errors.ClassicFallback()

    @staticmethod
    def _ensure_remote_build_supported(base: str) -> None:
        """Ensure the version of remote build is supported for the project.

        1. SNAPCRAFT_REMOTE_BUILD_STRATEGY must be unset, 'disable-fallback', or
          'force-fallback'
        2. core20 projects must use the legacy remote builder
        3. core24 and newer projects must use the craft-application remote builder

        :raises SnapcraftError: If the environment variable `SNAPCRAFT_REMOTE_BUILD_STRATEGY`
          is invalid.
        :raises SnapcraftError: If the remote build version cannot be used for the project.
        """
        build_strategy = os.environ.get("SNAPCRAFT_REMOTE_BUILD_STRATEGY", None)

        # 1. SNAPCRAFT_REMOTE_BUILD_STRATEGY must be unset, 'disable-fallback', or 'force-fallback'
        if build_strategy and build_strategy not in (
            "force-fallback",
            "disable-fallback",
        ):
            raise errors.SnapcraftError(
                message=(
                    f"Unknown value {build_strategy!r} in environment variable "
                    "'SNAPCRAFT_REMOTE_BUILD_STRATEGY'. "
                ),
                resolution=(
                    "Valid values are 'disable-fallback' and 'force-fallback'."
                ),
            )

        # 2. core20 projects must use the legacy remote builder (#4885)
        if base == "core20" and build_strategy == "disable-fallback":
            raise errors.SnapcraftError(
                message=(
                    "'SNAPCRAFT_REMOTE_BUILD_STRATEGY=disable-fallback' "
                    "cannot be used for core20 snaps."
                ),
                resolution=(
                    "Unset the environment variable or set it to 'force-fallback'."
                ),
            )

        # 3. core24 and newer projects must use the craft-application remote builder
        elif base not in ["core20", "core22"] and build_strategy == "force-fallback":
            raise errors.SnapcraftError(
                message=(
                    "'SNAPCRAFT_REMOTE_BUILD_STRATEGY=force-fallback' cannot "
                    "be used for core24 and newer snaps."
                ),
                resolution=(
                    "Unset the environment variable or set it to 'disable-fallback'."
                ),
            )

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

    @override
    def _set_global_environment(self, info: craft_parts.ProjectInfo) -> None:
        """Set global environment variables."""
        super()._set_global_environment(info)
        set_global_environment(info)


def create_app() -> Snapcraft:
    """Create a Snapcraft application with the proper commands."""
    snapcraft_services = services.SnapcraftServiceFactory(app=APP_METADATA)

    app = Snapcraft(
        app=APP_METADATA,
        services=snapcraft_services,
    )

    for group in cli.COMMAND_GROUPS:
        app.add_command_group(group.name, group.commands)

    return app


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
