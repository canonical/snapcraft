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
import time
from collections.abc import Collection, Mapping
from pathlib import Path
from typing import Any, cast

import craft_cli
import lazr.restfulclient.errors
from craft_application import errors
from craft_application.application import filter_plan
from craft_application.commands import ExtensibleCommand
from craft_application.errors import RemoteBuildError
from craft_application.launchpad.models import Build, BuildState
from craft_application.remote.utils import get_build_id
from craft_cli import emit
from overrides import overrides

from snapcraft import models
from snapcraft.const import SUPPORTED_ARCHS
from snapcraft.utils import confirm_with_user

_CONFIRMATION_PROMPT = (
    "All data sent to remote builders will be publicly available. "
    "Are you sure you want to continue?"
)


class RemoteBuildCommand(ExtensibleCommand):
    """Command passthrough for the remote-build command."""

    always_load_project = True
    name = "remote-build"
    help_msg = "Build a snap remotely on Launchpad."
    overview = textwrap.dedent(
        """
        Command remote-build sends the current project to be built
        remotely.  After the build is complete, packages for each
        architecture are retrieved and will be available in the
        local filesystem.

        If not specified in the snapcraft.yaml file, the list of
        architectures to build can be set using the --platforms option.
        If both are specified, an error will occur.

        Interrupted remote builds can be resumed using the --recover
        option, followed by the build number informed when the remote
        build was originally dispatched. The current state of the
        remote build for each architecture can be checked using the
        --status option.

        To set a timeout on the remote-build command, use the option
        ``--launchpad-timeout=<seconds>``. The timeout is local, so the build on
        launchpad will continue even if the local instance of snapcraft is
        interrupted or times out.
        """
    )

    @overrides
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--recover", action="store_true", help="recover an interrupted build"
        )
        parser.add_argument(
            "--launchpad-accept-public-upload",
            action="store_true",
            help="acknowledge that uploaded code will be publicly available.",
        )
        parser.add_argument(
            "--launchpad-timeout",
            type=int,
            default=0,
            metavar="<seconds>",
            help="Time in seconds to wait for launchpad to build.",
        )

        parser.add_argument(
            "--status", action="store_true", help="display remote build status"
        )
        parser.add_argument(
            "--build-id", metavar="build-id", help="specific build id to retrieve"
        )

        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            "--platform",
            type=str,
            metavar="name",
            default=os.getenv("CRAFT_PLATFORM"),
            help="Set platform to build for",
        )
        group.add_argument(
            "--build-for",
            type=str,
            metavar="arch",
            default=os.getenv("CRAFT_BUILD_FOR"),
            help="Set architecture to build for",
        )
        parser.add_argument(
            "--project", help="upload to the specified Launchpad project"
        )

    def _validate(self, parsed_args: argparse.Namespace) -> None:
        """Do pre-build validation."""
        if os.getenv("SUDO_USER") and os.geteuid() == 0:
            emit.progress(
                "Running with 'sudo' may cause permission errors and is discouraged.",
                permanent=True,
            )
            # Give the user a bit of time to process this before proceeding.
            time.sleep(1)

        if (
            not parsed_args.launchpad_accept_public_upload
            and (
                not parsed_args.project
                or not self._services.remote_build.is_project_private()
            )
            and not confirm_with_user(_CONFIRMATION_PROMPT, default=False)
        ):
            raise errors.RemoteBuildError(
                "Remote build needs explicit acknowledgement that data sent to build servers "
                "is public.",
                details=(
                    "In non-interactive runs, please use the option "
                    "`--launchpad-accept-public-upload`."
                ),
                reportable=False,
                retcode=77,
            )
        project = cast(models.Project, self._services.project)
        if project.architectures:
            for arch in project.architectures:
                if (
                    isinstance(arch, models.Architecture)
                    and arch.build_for
                    and "all" in arch.build_for
                ):
                    raise craft_cli.CraftError(
                        message="Remote build does not support architecture 'all'.",
                        resolution="Reconfigure your snap for architecture-dependent builds.",
                        reportable=False,
                        retcode=78,  # Configuration error
                    )

    def _run(self, parsed_args: argparse.Namespace, **kwargs: Any) -> int | None:
        """Run the remote-build command.

        :param parsed_args: Snapcraft's argument namespace.

        :raises AcceptPublicUploadError: If the user does not agree to upload data.
        """
        if parsed_args.project:
            self._services.remote_build.set_project(parsed_args.project)

        self._validate(parsed_args)
        emit.progress(
            "remote-build is experimental and is subject to change. Use with caution.",
            permanent=True,
        )

        builder = self._services.remote_build
        project = cast(models.Project, self._services.project)
        config = cast(dict[str, Any], self.config)
        project_dir = (
            Path(config.get("global_args", {}).get("project_dir") or ".")
            .expanduser()
            .resolve()
        )

        emit.trace(f"Project directory: {project_dir}")

        possible_build_plan = filter_plan(
            self._app.BuildPlannerClass.unmarshal(project.marshal()).get_build_plan(),
            platform=parsed_args.platform,
            build_for=parsed_args.build_for,
            host_arch=None,
        )

        architectures: list[str] | None = list(
            {build_info.build_for for build_info in possible_build_plan}
        )

        if parsed_args.platform and not architectures:
            emit.progress(
                f"platform '{parsed_args.platform}' is not present in the build plan.",
                permanent=True,
            )
            return 78  # Configuration error

        if parsed_args.build_for and not architectures:
            if parsed_args.build_for not in SUPPORTED_ARCHS:
                raise errors.RemoteBuildError(
                    f"build-for '{parsed_args.build_for}' is not supported.", retcode=78
                )
            # Allow the user to build for a single architecture if snapcraft.yaml
            # doesn't define architectures.
            architectures = [parsed_args.build_for]

        emit.debug(f"Architectures to build: {architectures}")

        if parsed_args.launchpad_timeout:
            emit.debug(f"Setting timeout to {parsed_args.launchpad_timeout} seconds")
            builder.set_timeout(parsed_args.launchpad_timeout)

        build_id = get_build_id(self._app.name, project.name, project_dir)
        if parsed_args.recover:
            emit.progress(f"Recovering build {build_id}")
            builds = builder.resume_builds(build_id)
        else:
            emit.progress(
                "Starting new build. It may take a while to upload large projects."
            )
            try:
                builds = builder.start_builds(
                    project_dir, architectures=architectures or None
                )
            except RemoteBuildError:
                emit.progress("Starting build failed.", permanent=True)
                emit.progress("Cleaning up")
                builder.cleanup()
                raise
            except lazr.restfulclient.errors.Conflict:
                emit.progress("Remote repository already exists.", permanent=True)
                emit.progress("Cleaning up")
                builder.cleanup()
                return 75

        try:
            returncode = self._monitor_and_complete(build_id, builds)
        except KeyboardInterrupt:
            if confirm_with_user("Cancel builds?", default=True):
                emit.progress("Cancelling builds.")
                builder.cancel_builds()
            returncode = 0
        except Exception:
            returncode = 1  # General error on any other exception
        if returncode != 75:  # TimeoutError
            emit.progress("Cleaning up")
            builder.cleanup()
        return returncode

    def _monitor_and_complete(  # noqa: PLR0912 (too many branches)
        self, build_id: str | None, builds: Collection[Build]
    ) -> int:
        builder = self._services.remote_build
        emit.progress("Monitoring build")
        states: Mapping[str, BuildState] = {}
        try:
            for states in builder.monitor_builds():
                building: set[str] = set()
                succeeded: set[str] = set()
                uploading: set[str] = set()
                not_building: set[str] = set()
                for arch, build_state in states.items():
                    if build_state.is_running:
                        building.add(arch)
                    elif build_state == BuildState.SUCCESS:
                        succeeded.add(arch)
                    elif build_state == BuildState.UPLOADING:
                        uploading.add(arch)
                    else:
                        not_building.add(arch)
                progress_parts: list[str] = []
                if not_building:
                    progress_parts.append("Stopped: " + ",".join(sorted(not_building)))
                if building:
                    progress_parts.append("Building: " + ", ".join(sorted(building)))
                if uploading:
                    progress_parts.append("Uploading: " + ",".join(sorted(uploading)))
                if succeeded:
                    progress_parts.append("Succeeded: " + ", ".join(sorted(succeeded)))
                emit.progress("; ".join(progress_parts))
        except TimeoutError:
            if build_id:
                resume_command = (
                    f"{self._app.name} remote-build --recover --build-id={build_id}"
                )
            else:
                resume_command = f"{self._app.name} remote-build --recover"
            emit.message(
                f"Timed out waiting for build.\nTo resume, run {resume_command!r}"
            )
            return 75  # Temporary failure

        return_code = 0

        for arch, build_state in states.items():
            if build_state == BuildState.FAILED:
                emit.progress(f"Build for architecture {arch} failed.", permanent=True)
                return_code = 1

        emit.progress(f"Fetching {len(builds)} build logs...")
        logs = builder.fetch_logs(Path.cwd())
        if not logs:
            return_code = 1
            emit.progress("No log files downloaded from Launchpad.", permanent=True)

        emit.progress("Fetching build artifacts...")
        artifacts = builder.fetch_artifacts(Path.cwd())
        if not artifacts:
            return_code = 1
            emit.progress(
                "No build artifacts downloaded from Launchpad.", permanent=True
            )

        log_names = sorted(path.name for path in logs.values() if path)
        artifact_names = sorted(path.name for path in artifacts)

        emit.message(
            "Build completed.\n"
            f"Log files: {', '.join(log_names)}\n"
            f"Artifacts: {', '.join(artifact_names)}"
        )
        return return_code
