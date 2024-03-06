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

import os
import pathlib
import signal
import sys
from typing import Any, List, Sequence

import craft_application.commands as craft_app_commands
import craft_application.models
import craft_cli
import pydantic
from craft_application import Application, AppMetadata, util
from craft_application.errors import CraftValidationError
from craft_application.models import BuildInfo
from craft_cli import emit
from craft_providers import bases
from overrides import override

from snapcraft import cli, commands, errors, models, services
from snapcraft.commands import unimplemented
from snapcraft.const import SUPPORTED_ARCHS, SnapArch
from snapcraft.extensions import apply_extensions
from snapcraft.models import Platform
from snapcraft.models.project import apply_root_packages
from snapcraft.providers import SNAPCRAFT_BASE_TO_PROVIDER_BASE
from snapcraft.utils import get_effective_base, get_host_architecture


class SnapcraftBuildPlanner(craft_application.models.BuildPlanner):
    """A project model that creates build plans."""

    base: str | None
    build_base: str | None = None
    name: str
    platforms: dict[str, Any] | None = None
    project_type: str | None = pydantic.Field(default=None, alias="type")

    @pydantic.validator("platforms")
    @classmethod
    def _validate_all_platforms(cls, platforms: dict[str, Any]) -> dict[str, Any]:
        """Validate and convert platform data to a dict of Platforms."""
        for platform_label in platforms:
            platform_data: dict[str, Any] = (
                platforms[platform_label] if platforms[platform_label] else {}
            )
            error_prefix = f"Error for platform entry '{platform_label}'"

            # Make sure the provided platform_set is valid
            try:
                platform = Platform(**platform_data)
            except CraftValidationError as err:
                raise CraftValidationError(f"{error_prefix}: {str(err)}") from None

            # build_on and build_for are validated
            # let's also validate the platform label
            if platform.build_on:
                build_on_one_of: Sequence[SnapArch | str] = platform.build_on
            else:
                build_on_one_of = [platform_label]

            # If the label maps to a valid architecture and
            # `build-for` is present, then both need to have the same value,
            # otherwise the project is invalid.
            if platform.build_for:
                build_target = platform.build_for[0]
                if platform_label in SUPPORTED_ARCHS and platform_label != build_target:
                    raise CraftValidationError(
                        str(
                            f"{error_prefix}: if 'build_for' is provided and the "
                            "platform entry label corresponds to a valid architecture, then "
                            f"both values must match. {platform_label} != {build_target}"
                        )
                    )
            # if no build-for is present, then the platform label needs to be a valid architecture
            elif platform_label not in SUPPORTED_ARCHS:
                raise CraftValidationError(
                    str(
                        f"{error_prefix}: platform entry label must correspond to a "
                        "valid architecture if 'build-for' is not provided."
                    )
                )

            # Both build and target architectures must be supported
            if not any(b_o in SUPPORTED_ARCHS for b_o in build_on_one_of):
                raise CraftValidationError(
                    str(
                        f"{error_prefix}: trying to build snap in one of "
                        f"{build_on_one_of}, but none of these build architectures are supported. "
                        f"Supported architectures: {SUPPORTED_ARCHS}"
                    )
                )

            platforms[platform_label] = platform

        return platforms

    def get_build_plan(self) -> List[BuildInfo]:
        """Get the build plan for this project."""
        build_infos: list[BuildInfo] = []
        effective_base = SNAPCRAFT_BASE_TO_PROVIDER_BASE[
            str(
                get_effective_base(
                    base=self.base,
                    build_base=self.build_base,
                    project_type=self.project_type,
                    name=self.name,
                )
            )
        ].value

        base = bases.BaseName("ubuntu", effective_base)

        if self.platforms is None:
            raise CraftValidationError("Must define at least one platform.")

        for platform_entry, platform in self.platforms.items():
            for build_for in platform.build_for or [platform_entry]:
                for build_on in platform.build_on or [platform_entry]:
                    build_infos.append(
                        BuildInfo(
                            platform=platform_entry,
                            build_on=build_on,
                            build_for=build_for,
                            base=base,
                        )
                    )

        return build_infos


APP_METADATA = AppMetadata(
    name="snapcraft",
    summary="Package, distribute, and update snaps for Linux and IoT",
    ProjectClass=models.Project,
    BuildPlannerClass=SnapcraftBuildPlanner,
    source_ignore_patterns=["*.snap"],
    project_variables=["version", "grade"],
    mandatory_adoptable_fields=["version", "summary", "description"],
)


MAPPED_ENV_VARS = {
    ev: "SNAP" + ev for ev in ("CRAFT_BUILD_FOR", "CRAFT_BUILD_ENVIRONMENT")
}


class Snapcraft(Application):
    """Snapcraft application definition."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Whether we know that we should use the core24-based codepath.
        self._known_core24 = False

        # Locate the project file. It's used in early execution to determine
        # compatibility with previous versions of the snapcraft codebase, and in
        # the package service to copy the project file into the snap payload if
        # manifest generation is enabled.
        try:
            self._snapcraft_yaml_path: pathlib.Path | None = self._resolve_project_path(
                None
            )
        except FileNotFoundError:
            self._snapcraft_yaml_path = None

        for craft_var, snapcraft_var in MAPPED_ENV_VARS.items():
            if env_val := os.getenv(snapcraft_var):
                os.environ[craft_var] = env_val

    @override
    def _configure_services(self, platform: str | None, build_for: str | None) -> None:
        if build_for is None:
            build_for = util.get_host_architecture()

        self.services.set_kwargs(
            "package",
            platform=platform,
            build_for=build_for,
            snapcraft_yaml_path=self._snapcraft_yaml_path,
        )

        super()._configure_services(platform, build_for)

    @property
    def command_groups(self):
        """Short-circuit the standard command groups for now."""
        # TODO: Remove this once we've got lifecycle commands and version migrated.
        return self._command_groups

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
    def _extra_yaml_transform(
        self, yaml_data: dict[str, Any], *, build_on: str, build_for: str | None
    ) -> dict[str, Any]:
        arch = build_on
        target_arch = build_for if build_for else get_host_architecture()
        new_yaml_data = apply_extensions(yaml_data, arch=arch, target_arch=target_arch)
        return apply_root_packages(new_yaml_data)

    @override
    def _get_dispatcher(self) -> craft_cli.Dispatcher:
        """Configure this application. Should be called by the run method.

        Side-effect: This method may exit the process.

        :returns: A ready-to-run Dispatcher object
        """
        # Handle "multiplexing" of Snapcraft "codebases" depending on the
        # project's base (if any). Here, we handle the case where there *is*
        # a project and it's core24, which means it should definitely fall into
        # the craft-application-based flow.
        if self._snapcraft_yaml_path:
            with self._snapcraft_yaml_path.open() as file:
                yaml_data = util.safe_yaml_load(file)
            base = yaml_data.get("base")
            build_base = yaml_data.get("build-base")
            if "core24" in (base, build_base) or build_base == "devel":
                # We know for sure that we're handling a core24 project
                self._known_core24 = True
            elif any(arg in ("version", "--version", "-V") for arg in sys.argv):
                pass
            else:
                raise errors.ClassicFallback()

        dispatcher = craft_cli.Dispatcher(
            self.app.name,
            self.command_groups,
            summary=str(self.app.summary),
            extra_global_args=self._global_arguments,
            # TODO: craft-application should allow setting the default command without
            # overriding `_get_dispatcher()`
            default_command=craft_app_commands.lifecycle.PackCommand,
        )

        try:
            craft_cli.emit.trace("pre-parsing arguments...")
            # Workaround for the fact that craft_cli requires a command.
            # https://github.com/canonical/craft-cli/issues/141
            if any(arg in ("--version", "-V") for arg in sys.argv) and (
                "version" not in sys.argv
            ):
                global_args = dispatcher.pre_parse_args(["version", *sys.argv[1:]])
            else:
                global_args = dispatcher.pre_parse_args(sys.argv[1:])
        except craft_cli.ProvideHelpException as err:
            print(err, file=sys.stderr)  # to stderr, as argparse normally does
            craft_cli.emit.ended_ok()
            sys.exit(0)
        except craft_cli.ArgumentParsingError as err:
            print(err, file=sys.stderr)  # to stderr, as argparse normally does
            craft_cli.emit.ended_ok()
            sys.exit(64)  # Command line usage error from sysexits.h
        except KeyboardInterrupt as err:
            self._emit_error(craft_cli.CraftError("Interrupted."), cause=err)
            sys.exit(128 + signal.SIGINT)
        # pylint: disable-next=broad-exception-caught
        except Exception as err:  # noqa: BLE001
            self._emit_error(
                craft_cli.CraftError(
                    f"Internal error while loading {self.app.name}: {err!r}"
                )
            )
            if os.getenv("CRAFT_DEBUG") == "1":
                raise
            sys.exit(70)  # EX_SOFTWARE from sysexits.h

        craft_cli.emit.trace("Preparing application...")
        self.configure(global_args)

        return dispatcher


def create_app() -> Snapcraft:
    """Create a Snapcraft application with the proper commands."""
    snapcraft_services = services.SnapcraftServiceFactory(app=APP_METADATA)

    app = Snapcraft(
        app=APP_METADATA,
        services=snapcraft_services,
        extra_loggers={"snapcraft.remote"},
    )

    app.add_command_group(
        "Lifecycle",
        [
            craft_app_commands.lifecycle.CleanCommand,
            craft_app_commands.lifecycle.PullCommand,
            craft_app_commands.lifecycle.BuildCommand,
            craft_app_commands.lifecycle.StageCommand,
            craft_app_commands.lifecycle.PrimeCommand,
            craft_app_commands.lifecycle.PackCommand,
            commands.SnapCommand,  # Hidden (legacy compatibility)
            unimplemented.RemoteBuild,
            unimplemented.Plugins,
            unimplemented.ListPlugins,
            unimplemented.Try,
        ],
    )
    app.add_command_group(
        "Extensions",
        [
            commands.ListExtensions,
            commands.ExpandExtensions,
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
        list(craft_app_commands.get_other_command_group().commands)
        + [
            unimplemented.Lint,
            unimplemented.Init,
        ],
    )

    return app


def main() -> int:
    """Run craft-application based snapcraft with classic fallback."""
    app = create_app()

    try:
        return app.run()
    except errors.ClassicFallback:
        emit.debug("Falling back from craft-application to snapcraft.")
        return cli.run()
