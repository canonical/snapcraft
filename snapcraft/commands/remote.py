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
from craft_application.application import filter_plan
from craft_application.commands import RemoteBuild
from craft_application.util import humanize_list, safe_yaml_load
from craft_cli import emit
from craft_platforms import DebianArchitecture
from overrides import override

from snapcraft import errors, models
from snapcraft.const import SUPPORTED_ARCHS
from snapcraft.parts import yaml_utils


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

        project = cast(models.Project, self._services.project)
        build_plan = self._app.BuildPlannerClass.unmarshal(
            project.marshal()
        ).get_build_plan()

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

        _validate_build_for_and_platforms(parsed_args.remote_build_build_fors)

    @override
    def _get_build_args(self, parsed_args: argparse.Namespace) -> dict[str, Any]:
        project = cast(models.Project, self._services.project)
        build_plan = self._app.BuildPlannerClass.unmarshal(
            project.marshal()
        ).get_build_plan()
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


def _validate_build_for_and_platforms(build_for: list[str]):
    """Ensure --build-for and shorthand platforms are not used together.

    This combination fails in Launchpad with an unhelpful error, so Snapcraft will
    fail early with a user-friendly error. See LP#2077005 and LP#2098811 for details.

    This validator can be removed when LP#2077005 and LP#2098811 are resolved or when
    Launchpad uses craft-platforms to orchestrate snap builds.

    :param build_for: The list of architectures to build for from the command line.

    :raises RemoteBuildError: If --build-for and shorthand platforms are used together.
    """
    if not build_for:
        return

    try:
        project_file = yaml_utils.get_snap_project().project_file
        with project_file.open() as file:
            data = safe_yaml_load(file)
    except (errors.ProjectMissing, craft_application.errors.YamlError) as err:
        # the project has to exist by this point, but this is a non-critical
        # validator so log the problem and move on
        emit.debug("Couldn't find or parse the project's snapcraft.yaml")
        emit.debug(f"Error: {err}")
        return

    if data.get("platforms") and any(not v for v in data["platforms"].values()):
        raise craft_application.errors.RemoteBuildError(
            message=(
                "Launchpad can't build snaps when using '--build-for' with "
                "a shorthand platforms entry in the project's snapcraft.yaml."
            ),
            resolution=(
                "Use full platform entries or remove the '--build-for' argument."
            ),
            doc_slug="/explanation/remote-build.html",
            retcode=os.EX_CONFIG,
        )
