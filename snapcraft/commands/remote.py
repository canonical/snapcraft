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
from typing import Any, cast

import craft_application.errors
from craft_application import errors
from craft_application.application import filter_plan
from craft_application.commands import RemoteBuild
from craft_application.util import humanize_list
from craft_cli import emit
from craft_platforms import DebianArchitecture
from overrides import override

from snapcraft import models
from snapcraft.const import SUPPORTED_ARCHS

_CONFIRMATION_PROMPT = (
    "All data sent to remote builders will be publicly available. "
    "Are you sure you want to continue?"
)

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
        print(parsed_args)

        for build_for in cast(list[str], parsed_args.remote_build_build_fors) or []:
            if build_for not in [*SUPPORTED_ARCHS, "all"]:
                raise errors.RemoteBuildError(
                    f"Unsupported build-for architecture {build_for!r}",
                    resolution=(
                        "Use a supported Debian architecture. Supported "
                        f"architectures are: {humanize_list(SUPPORTED_ARCHS, 'and')}"
                    ),
                    doc_slug="/explanation/remote-build.html",
                    retcode=os.EX_CONFIG,
                )

        project = cast(models.Project, self._services.project)
        build_plan = self._app.BuildPlannerClass.unmarshal(project.marshal()).get_build_plan()

        # mapping of `build-on` to `build-for` architectures
        build_map: dict[str, list[str]] = {}
        for build_info in build_plan:
            build_map.setdefault(build_info.build_on, []).append(build_info.build_for)

        # assemble a list so all errors are shown at once
        build_on_errors = []
        for build_on, build_fors in build_map.items():
            if len(build_fors) > 1:
                build_on_errors.append(
                    f"\n  - Building on {build_on!r} will create snaps for "
                    f"{humanize_list(build_fors, 'and')}."
                )

        if build_on_errors:
            raise errors.RemoteBuildError(
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
        project = cast(models.Project, self._services.project)
        build_plan = self._app.BuildPlannerClass.unmarshal(project.marshal()).get_build_plan()
        build_fors = cast(list[str], parsed_args.remote_build_build_fors)
        archs: list[str] = []
        if project.platforms or project._architectures_in_yaml:
            # if the project has platforms, then `--build-for` acts as a filter
            if build_fors:
                emit.debug("Filtering the build plan using the '--build-for' argument.")
                for build_for in build_fors:
                    filtered_build_plan = filter_plan(
                        build_plan,
                        platform=None,
                        build_for=build_for,
                        host_arch=None,
                    )
                    archs.extend([info.build_for for info in filtered_build_plan])
                    if not archs:
                        raise craft_application.errors.EmptyBuildPlanError()
            else:
                emit.debug("Using the project's build plan")
                archs = [build_info.build_for for build_info in build_plan]
        # No architectures in the project means '--build-for' no longer acts as a filter.
        # Instead, it defines the architectures to build for.
        elif build_fors:
            emit.debug("Using '--build-for' as the list of architectures to build for")
            archs = build_fors
        # default is to build for the host architecture
        else:
            archs = [str(DebianArchitecture.from_host())]
            emit.debug(
                f"Using host architecture {archs[0]} because no architectures were "
                "defined in the project or as a command-line argument."
            )

        emit.debug(f"Architectures to build for: {humanize_list(archs, 'and')}")
        return {"architectures": archs}
