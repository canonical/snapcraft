# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Snapcraft remote build command."""

import argparse
import os
import textwrap

from craft_cli import BaseCommand, emit
from craft_cli.helptexts import HIDDEN
from overrides import overrides

from snapcraft.legacy_cli import run_legacy
from snapcraft.parts.lifecycle import get_snap_project, process_yaml
from snapcraft.utils import confirm_with_user
from snapcraft_legacy.internal.remote_build.errors import AcceptPublicUploadError

_CONFIRMATION_PROMPT = (
    "All data sent to remote builders will be publicly available. "
    "Are you sure you want to continue?"
)


class RemoteBuildCommand(BaseCommand):
    """Command passthrough for the remote-build command."""

    name = "remote-build"
    help_msg = "Dispatch a snap for remote build"
    overview = textwrap.dedent(
        """
        Command remote-build sends the current project to be built
        remotely.  After the build is complete, packages for each
        architecture are retrieved and will be available in the
        local filesystem.

        If not specified in the snapcraft.yaml file, the list of
        architectures to build can be set using the --build-on option.
        If both are specified, an error will occur.

        Interrupted remote builds can be resumed using the --recover
        option, followed by the build number informed when the remote
        build was originally dispatched. The current state of the
        remote build for each architecture can be checked using the
        --status option."""
    )

    @overrides
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--recover", action="store_true", help="recover an interrupted build"
        )
        parser.add_argument(
            "--status", action="store_true", help="display remote build status"
        )
        parser_target = parser.add_mutually_exclusive_group()
        parser_target.add_argument(
            "--build-on",
            metavar="arch",
            nargs="+",
            help=HIDDEN,
        )
        parser_target.add_argument(
            "--build-for",
            metavar="arch",
            nargs="+",
            help="architecture to build for",
        )
        parser.add_argument(
            "--build-id", metavar="build-id", help="specific build id to retrieve"
        )
        parser.add_argument(
            "--launchpad-accept-public-upload",
            action="store_true",
            help="acknowledge that uploaded code will be publicly available.",
        )

    @overrides
    def run(self, parsed_args):
        if os.getenv("SUDO_USER") and os.geteuid() == 0:
            emit.message(
                "Running with 'sudo' may cause permission errors and is discouraged."
            )

        emit.message(
            "snapcraft remote-build is experimental and is subject to change - use with caution."
        )

        if parsed_args.build_on:
            emit.message("Use --build-for instead of --build-on")
            parsed_args.build_for = parsed_args.build_on

        if not parsed_args.launchpad_accept_public_upload and not confirm_with_user(
            _CONFIRMATION_PROMPT
        ):
            raise AcceptPublicUploadError()

        snap_project = get_snap_project()
        # TODO proper core22 support would mean we need to load the project
        # yaml_data = process_yaml(snap_project.project_file)
        # for now, only log explicitly that we are falling back to legacy to
        # remote build for core22
        process_yaml(snap_project.project_file)

        emit.debug(
            "core22 not yet supported in new code base: re-executing into legacy for remote-build"
        )
        run_legacy()
