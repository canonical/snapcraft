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
from collections.abc import Collection
from pathlib import Path
from typing import Any, cast

from craft_application.application import _filter_plan
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
            "--recover", action="store_true", help="recover an interrupted build"
        )
        parser.add_argument(
            "--status", action="store_true", help="display remote build status"
        )
        parser.add_argument(
            "--build-id", metavar="build-id", help="specific build id to retrieve"
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

    # pylint: disable=too-many-branches, too-many-statements
    def _run(self, parsed_args: argparse.Namespace, **kwargs: Any) -> int | None:
        """Run the remote-build command.

        :param parsed_args: Snapcraft's argument namespace.

        :raises AcceptPublicUploadError: If the user does not agree to upload data.
        """
        if os.getenv("SUDO_USER") and os.geteuid() == 0:
            emit.progress(
                "Running with 'sudo' may cause permission errors and is discouraged.",
                permanent=True,
            )
            # Give the user a bit of time to process this before proceeding.
            time.sleep(1)

        emit.progress(
            "remote-build is experimental and is subject to change. Use with caution.",
            permanent=True,
        )

        if not parsed_args.launchpad_accept_public_upload and not confirm_with_user(
            _CONFIRMATION_PROMPT, default=False
        ):
            emit.progress(
                "Remote build needs explicit acknowledgement that data sent to build servers is "
                "public.\n"
                "In non-interactive runs, please use the option "
                "`--launchpad-accept-public-upload`.",
                permanent=True,
            )
            return 77

        builder = self._services.remote_build
        project = cast(models.Project, self._services.project)
        config = cast(dict[str, Any], self.config)
        project_dir = (
            Path(config.get("global_args", {}).get("project_dir") or ".")
            .expanduser()
            .resolve()
        )

        emit.trace(f"Project directory: {project_dir}")

        build_planner = self._app.BuildPlannerClass.unmarshal(project.marshal())
        full_build_plan = build_planner.get_build_plan()
        possible_build_plan = _filter_plan(
            full_build_plan,
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
            if parsed_args.build_for in SUPPORTED_ARCHS:
                # allow the user to build for a single architecture
                # if the snapcraft.yaml not defined the platforms
                architectures = [parsed_args.build_for]
            else:
                emit.progress(
                    f"build-for '{parsed_args.build_for}' is not supported.",
                    permanent=True,
                )
                return 78  # Configuration error

        emit.debug(f"Architectures to build: {architectures}")

        if not architectures:
            architectures = None

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
                builds = builder.start_builds(project_dir, architectures=architectures)
            except RemoteBuildError:
                emit.progress("Starting build failed.", permanent=True)
                emit.progress("Cleaning up")
                builder.cleanup()
                raise

        try:
            returncode = self._monitor_and_complete(build_id, builds)
        except KeyboardInterrupt:
            if confirm_with_user("Cancel builds?", default=True):
                emit.progress("Cancelling builds.")
                builder.cancel_builds()
                emit.progress("Cleaning up")
                builder.cleanup()
            returncode = 0
        else:
            if returncode != 75:  # TimeoutError
                emit.progress("Cleaning up")
                builder.cleanup()
        return returncode

    def _monitor_and_complete(
        self, build_id: str | None, builds: Collection[Build]
    ) -> int:
        builder = self._services.remote_build
        emit.progress("Monitoring build")
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

        emit.progress(f"Fetching {len(builds)} build logs...")
        logs = builder.fetch_logs(Path.cwd())

        emit.progress("Fetching build artifacts...")
        artifacts = builder.fetch_artifacts(Path.cwd())

        log_names = sorted(path.name for path in logs.values() if path)
        artifact_names = sorted(path.name for path in artifacts)

        emit.message(
            "Build completed.\n"
            f"Log files: {', '.join(log_names)}\n"
            f"Artifacts: {', '.join(artifact_names)}"
        )
        return 0
