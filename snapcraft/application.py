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
from typing import Any, List, cast

import craft_application.commands as craft_app_commands
import craft_application.models
import craft_cli
import pydantic
from craft_application import Application, AppMetadata, util
from craft_application.models import BuildInfo
from craft_cli import emit
from craft_providers import bases
from overrides import override

from snapcraft import cli, commands, errors, models, services
from snapcraft.commands import unimplemented
from snapcraft.extensions import apply_extensions
from snapcraft.models import Architecture
from snapcraft.models.project import root_packages_transform, validate_architectures
from snapcraft.providers import SNAPCRAFT_BASE_TO_PROVIDER_BASE
from snapcraft.utils import get_effective_base, get_host_architecture


class SnapcraftBuildPlanner(craft_application.models.BuildPlanner):
    """A project model that creates build plans."""

    architectures: List[str | Architecture] = pydantic.Field(
        default_factory=lambda: [get_host_architecture()]
    )
    base: str | None = None
    build_base: str | None = None
    name: str | None = None
    project_type: str | None = None

    @pydantic.validator("architectures", always=True)
    def _validate_architecture_data(  # pylint: disable=no-self-argument
        cls, architectures: list[str | Architecture]
    ) -> list[Architecture]:
        """Validate architecture data.

        Most importantly, converts architecture strings into Architecture objects.
        """
        return validate_architectures(architectures)

    def get_build_plan(self) -> List[BuildInfo]:
        """Get the build plan for this project."""
        build_plan: List[BuildInfo] = []

        for arch in self.architectures:
            # build_for will be a single element list
            if isinstance(arch, Architecture):
                build_on = arch.build_on
                build_for = cast(list[str], arch.build_for)[0]
            else:
                build_on = arch
                build_for = arch

            # TODO: figure out when to filter `all`
            if build_for == "all":
                build_for = get_host_architecture()

            # build on will be a list of archs
            for build_on in cast(list[str], build_on):
                base = SNAPCRAFT_BASE_TO_PROVIDER_BASE[
                    str(
                        get_effective_base(
                            base=self.base,
                            build_base=self.build_base,
                            name=self.name,
                            project_type=self.project_type,
                        )
                    )
                ]
                build_plan.append(
                    BuildInfo(
                        platform=f"ubuntu@{base.value}",
                        build_on=build_on,
                        build_for=build_for,
                        base=bases.BaseName("ubuntu", base.value),
                    )
                )

        return build_plan


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

        for craft_var, snapcraft_var in MAPPED_ENV_VARS.items():
            if env_val := os.getenv(snapcraft_var):
                os.environ[craft_var] = env_val

    @override
    def _configure_services(self, platform: str | None, build_for: str | None) -> None:
        if build_for is None:
            build_for = util.get_host_architecture()

        self.services.set_kwargs("package", platform=platform, build_for=build_for)
        super()._configure_services(platform, build_for)

    @property
    def command_groups(self):
        """Short-circuit the standard command groups for now."""
        # TODO: Remove this once we've got lifecycle commands and version migrated.
        return self._command_groups

    def _resolve_project_path(self, project_dir: pathlib.Path | None) -> pathlib.Path:
        """Overridden to handle the two possible locations for snapcraft.yaml."""
        if project_dir is None:
            project_dir = pathlib.Path.cwd()

        try:
            return super()._resolve_project_path(project_dir / "snap")
        except FileNotFoundError:
            return super()._resolve_project_path(project_dir)

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
        return root_packages_transform(new_yaml_data)

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
        try:
            existing_project = self._resolve_project_path(None)
        except FileNotFoundError:
            # No project file - don't know if we should use core24 code or not.
            pass
        else:
            with existing_project.open() as file:
                yaml_data = util.safe_yaml_load(file)
            base = yaml_data.get("base")
            build_base = yaml_data.get("build-base")
            if "core24" in (base, build_base) or build_base == "devel":
                # We know for sure that we're handling a core24 project
                self._known_core24 = True
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
            if "--version" in sys.argv or "-V" in sys.argv:
                try:
                    global_args = dispatcher.pre_parse_args(["pull", *sys.argv[1:]])
                except craft_cli.ArgumentParsingError:
                    global_args = dispatcher.pre_parse_args(sys.argv[1:])
            else:
                global_args = dispatcher.pre_parse_args(sys.argv[1:])

            if global_args.get("version"):
                craft_cli.emit.ended_ok()
                print(f"{self.app.name} {self.app.version}")
                sys.exit(0)
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
        [
            unimplemented.Version,
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
