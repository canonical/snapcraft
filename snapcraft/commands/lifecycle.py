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
import os
import textwrap
from typing import Any, cast

import craft_application.commands
from craft_cli import emit
from typing_extensions import override

import snapcraft.errors
import snapcraft.pack
from snapcraft import errors
from snapcraft.models.project import Project


def _add_ua_args(parser: argparse.ArgumentParser) -> None:
    """Add hidden UA args."""
    parser.add_argument(
        "--ua-token",
        type=str,
        metavar="ua-token",
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--enable-experimental-ua-services",
        action="store_true",
        default=False,
        help=argparse.SUPPRESS,
    )


def _validate_ua_args(parsed_args: argparse.Namespace) -> None:
    """Validate if UA args or environment variables are used.

    Using a UA command line argument is an error, but the UA environment variable
    will only emit a warning.
    """
    if os.environ.get("SNAPCRAFT_UA_TOKEN"):
        emit.warning(
            "Ignoring the 'SNAPCRAFT_UA_TOKEN' environment variable. "
            "The Pro token attached to the host will be used instead."
        )

    if parsed_args.ua_token is not None:
        raise errors.SnapcraftError(
            "'--ua-token' is not supported for this base.",
            details="The Pro token attached to the host is used instead.",
            resolution="Remove the '--ua-token' argument.",
        )

    if parsed_args.enable_experimental_ua_services:
        raise errors.SnapcraftError(
            "Pro support is stable for this base.",
            resolution="Remove the '--enable-experimental-ua-services' argument.",
        )


class _UAServicesMixin(craft_application.commands.base.ExtensibleCommand):
    """Mixin to add UA args and validation to lifecycle commands."""

    @override
    def _fill_parser(self, parser: argparse.ArgumentParser) -> None:
        super()._fill_parser(parser)  # type: ignore[misc]
        _add_ua_args(parser)

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        _validate_ua_args(parsed_args)
        super()._run(parsed_args, step_name=step_name, **kwargs)  # type: ignore[misc]


class PullCommand(_UAServicesMixin, craft_application.commands.lifecycle.PullCommand):
    """Snapcraft pull command."""


class BuildCommand(_UAServicesMixin, craft_application.commands.lifecycle.BuildCommand):
    """Snapcraft build command."""


class StageCommand(_UAServicesMixin, craft_application.commands.lifecycle.StageCommand):
    """Snapcraft stage command."""


class PrimeCommand(_UAServicesMixin, craft_application.commands.lifecycle.PrimeCommand):
    """Snapcraft prime command."""


class CleanCommand(_UAServicesMixin, craft_application.commands.lifecycle.CleanCommand):
    """Snapcraft clean command."""


class PackCommand(_UAServicesMixin, craft_application.commands.lifecycle.PackCommand):
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
            # Skip the mixin's _run to avoid validating UA args twice.
            _validate_ua_args(parsed_args)
            craft_application.commands.lifecycle.PackCommand._run(self, parsed_args)

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


class TryCommand(PackCommand):
    """Prepare the parts for ``snap try``."""

    name = "try"
    help_msg = 'Prepare a snap for "snap try".'
    overview = textwrap.dedent(
        """
        Process parts and expose the ``prime`` directory containing the
        final payload, in preparation for ``snap try prime``.
        """
    )

    @override
    def run_managed(self, parsed_args: argparse.Namespace) -> bool:
        """Overridden to return false, such that the command fails early."""
        return False

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        project = cast(Project, self.services.get("project").get())
        effective_base = project.get_effective_base()

        raise snapcraft.errors.FeatureNotImplemented(
            f"'snapcraft try' is not implemented for {effective_base!r}"
        )


class SnapCommand(PackCommand):
    """Removed command to pack the final snap payload."""

    name = "snap"
    hidden = True

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)
