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

"""Snapcraft lifecycle commands."""

import argparse
import textwrap
from typing import Any

import craft_application.commands
from craft_cli import emit
from typing_extensions import override

import snapcraft.pack


class PackCommand(craft_application.commands.lifecycle.PackCommand):
    """Snapcraft pack command."""

    name = "pack"
    help_msg = "Create the final artifact"
    overview = textwrap.dedent(
        """
        Process parts and create a snap file containing the project payload
        with the provided metadata. If a directory is specified, pack its
        contents instead.
        """
    )

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        """Add arguments specific to the pack command."""
        super()._fill_parser(parser)

        parser.add_argument(
            "directory",
            metavar="directory",
            type=str,
            nargs="?",
            default=None,
            help="Directory to pack",
        )

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        """Pack a directory or run the lifecycle and pack all artifacts."""
        if parsed_args.directory:
            emit.progress("Packing...")
            snap_filename = snapcraft.pack.pack_snap(
                parsed_args.directory, output=str(parsed_args.output)
            )
            emit.message(f"Packed {snap_filename}")
        else:
            super()._run(parsed_args)

    @override
    def needs_project(self, parsed_args: argparse.Namespace) -> bool:
        """Project is not required to pack a directory."""
        if parsed_args.directory:
            emit.debug("Not loading project because a directory was provided.")
            return False

        emit.debug("Loading project because a directory was not provided.")
        return True

    @override
    def run_managed(self, parsed_args: argparse.Namespace) -> bool:
        """Return whether the command should run in managed mode or not.

        Packing a directory always runs locally.
        """
        if parsed_args.directory:
            emit.debug("Not running managed mode because a directory was provided.")
            return False

        return super().run_managed(parsed_args)


class SnapCommand(PackCommand):
    """Deprecated legacy command to pack the final snap payload."""

    name = "snap"
    hidden = True

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        emit.progress(
            "Warning: the 'snap' command is deprecated and will be removed "
            "in a future release of Snapcraft. Use 'pack' instead.",
            permanent=True,
        )

        super()._run(parsed_args, step_name, **kwargs)
