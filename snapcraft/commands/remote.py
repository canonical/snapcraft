# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Snapcraft remote build command that using craft-application."""

import argparse
import os
import textwrap
from typing import Any, Literal, cast

import craft_application.errors
import craft_platforms
from craft_application.commands import RemoteBuild
from craft_application.util import humanize_list
from craft_cli import emit
from craft_platforms import DebianArchitecture
from overrides import override

from snapcraft.const import SUPPORTED_ARCHS
from snapcraft.services import BuildPlan


class RemoteBuildCommand(RemoteBuild):
    """Command passthrough for the remote-build command."""

    help_msg = "Build a snap remotely on Launchpad."
    overview = textwrap.dedent(
        """
        Command remote-build sends the current project to be built
        remotely.  After the build is complete, packages for each
        architecture are retrieved and will be available in the
        local filesystem.

        If the project contains a ``platforms`` or ``architectures`` key,
        then the project's build plan is used. The build plan can be filtered
        using the ``--build-for`` argument.

        If the project doesn't contain a ``platforms`` or ``architectures`` key,
        then the architectures to build for are defined by the ``--build-for``
        argument.

        If there are no architectures defined in the project file or with
        ``--build-for``, then the default behavior is to build for the host
        architecture of the local machine.

        Interrupted remote builds can be resumed using the --recover
        option.

        To set a timeout on the remote-build command, use the option
        ``--launchpad-timeout=<seconds>``. The timeout is local, so the build on
        launchpad will continue even if the local instance of snapcraft is
        interrupted or times out.
        """
    )

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super()._fill_parser(parser)

        parser.add_argument(
            "--build-for",
            type=lambda arg: [arch.strip() for arch in arg.split(",")],
            metavar="arch",
            default=os.getenv("CRAFT_BUILD_FOR"),
            help="Comma-separated list of architectures to build for",
            # '--build-for' needs to be handled differently since remote-build can
            # build for architecture that is not in the project metadata
            dest="remote_build_build_fors",
        )

    @override
    def _pre_build(self, parsed_args: argparse.Namespace):
        """Perform pre-build validation.

        :param parsed_args: Argument namespace to validate
        :raises RemoteBuildError: If an unsupported architecture is specified, or multiple
        artifacts will be created for the same build-on.
        """
        for build_for in cast(list[str], parsed_args.remote_build_build_fors) or []:
            if build_for not in [*SUPPORTED_ARCHS, "all"]:
                raise craft_application.errors.RemoteBuildError(
                    f"Unsupported build-for architecture {build_for!r}",
                    resolution=(
                        "Use a supported Debian architecture. Supported "
                        f"architectures are: {humanize_list(SUPPORTED_ARCHS, 'and')}"
                    ),
                    doc_slug="/explanation/remote-build.html",
                    retcode=os.EX_CONFIG,
                )

        build_plan_service = cast(BuildPlan, self._services.get("build_plan"))
        build_plan = build_plan_service.create_launchpad_build_plan(
            platforms=None, build_for=None, build_on=None
        )

        # mapping of `build-on` to `build-for` architectures
        build_map: dict[str, list[str]] = {}
        for build_info in build_plan:
            build_map.setdefault(build_info.build_on, []).append(build_info.build_for)

        # assemble a list so all errors are shown at once
        build_on_errors = []
        for build_on, build_fors in build_map.items():
            if len(build_fors) > 1:
                build_on_errors.append(
                    f"\n  - Building on {build_on} will create snaps for "
                    f"{humanize_list(build_fors, 'and', item_format='{!s}')}."
                )

        if build_on_errors:
            raise craft_application.errors.RemoteBuildError(
                message=(
                    "Remote build does not support building multiple snaps on the "
                    f"same architecture:{''.join(build_on_errors)}"
                ),
                resolution=(
                    "Ensure that only one snap will be created for each build-on "
                    "architecture."
                ),
                doc_slug="/explanation/remote-build.html",
                retcode=os.EX_CONFIG,
            )

    @override
    def _get_build_args(self, parsed_args: argparse.Namespace) -> dict[str, Any]:
        project = self._services.get("project").get_raw()
        if parsed_args.remote_build_build_fors:
            build_fors: list[DebianArchitecture | Literal["all"]] | None = [
                "all" if arch == "all" else craft_platforms.DebianArchitecture(arch)
                for arch in parsed_args.remote_build_build_fors
            ]
        else:
            build_fors = None

        archs: list[DebianArchitecture | Literal["all"]] = []
        if project.get("platforms") or project.get("architectures"):
            build_plan_service = cast(BuildPlan, self._services.get("build_plan"))
            build_plan = build_plan_service.create_launchpad_build_plan(
                platforms=None, build_for=build_fors or None, build_on=None
            )
            # if the project has platforms, then `--build-for` acts as a filter
            if build_fors:
                emit.debug("Filtering the build plan using the '--build-for' argument.")
                # Launchpad's API only accepts a list of architectures but doesn't
                # have a concept of 'build-on' vs 'build-for'.
                # Passing the build-on archs is safe because:
                # * `_pre_build()` ensures no more than one artifact can be built on each build-on arch.
                # * Launchpad chooses one arch if the same artifact can be built on multiple archs.
                archs.extend([info.build_on for info in build_plan])
                if not archs:
                    raise craft_application.errors.EmptyBuildPlanError()
            else:
                emit.debug("Using the project's build plan")
                archs = [build_info.build_on for build_info in build_plan]
        # No architectures in the project means '--build-for' no longer acts as a filter.
        # Instead, it defines the architectures to build on, and for.
        elif build_fors:
            emit.debug("Using '--build-for' as the list of architectures to build for")
            archs = build_fors
        # default is to build on, and for, the host architecture
        else:
            archs = [DebianArchitecture.from_host()]
            emit.debug(
                f"Using host architecture {archs[0]} because no architectures were "
                "defined in the project or as a command-line argument."
            )

        emit.debug(f"Architectures to build for: {humanize_list(archs, 'and')}")
        return {"architectures": archs}
