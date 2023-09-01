# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

from snapcraft.errors import MaintenanceBase, SnapcraftError
from snapcraft.legacy_cli import run_legacy
from snapcraft.parts import yaml_utils
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

    def _get_base(self) -> str:
        """Get a valid base from the project's snapcraft.yaml.

        :returns: The project's base.

        :raises SnapcraftError: If the base is unknown or missing.
        :raises MaintenanceBase: If the base is not supported
        """
        snapcraft_yaml = yaml_utils.get_snap_project().project_file

        with open(snapcraft_yaml, encoding="utf-8") as file:
            base = yaml_utils.get_base(file)

        if base is None:
            raise SnapcraftError(
                f"Could not determine base from {str(snapcraft_yaml)!r}."
            )

        emit.debug(f"Got base {base!r} from {str(snapcraft_yaml)!r}.")

        if base in yaml_utils.ESM_BASES:
            raise MaintenanceBase(base)

        if base not in yaml_utils.BASES:
            raise SnapcraftError(f"Unknown base {base!r} in {str(snapcraft_yaml)!r}.")

        return base

    def _run_remote_build(self, base: str) -> None:
        # bases newer than core22 must use the new remote-build
        if base in yaml_utils.CURRENT_BASES - {"core22"}:
            emit.debug(
                "Using fallback remote-build because new remote-build is not available."
            )
            # TODO: use new remote-build code (#4323)
            run_legacy()
            return

        emit.debug("Running fallback remote-build.")
        run_legacy()

    @overrides
    def run(self, parsed_args) -> None:
        if os.getenv("SUDO_USER") and os.geteuid() == 0:
            emit.message(
                "Running with 'sudo' may cause permission errors and is discouraged."
            )

        emit.message(
            "snapcraft remote-build is experimental and is subject to change "
            "- use with caution."
        )

        if parsed_args.build_on:
            emit.message("Use --build-for instead of --build-on")
            parsed_args.build_for = parsed_args.build_on

        if not parsed_args.launchpad_accept_public_upload and not confirm_with_user(
            _CONFIRMATION_PROMPT
        ):
            raise AcceptPublicUploadError()

        base = self._get_base()
        self._run_remote_build(base)
