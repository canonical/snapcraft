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

"""Snapcraft lifecycle commands."""

import abc
import textwrap
from typing import TYPE_CHECKING

from craft_cli import BaseCommand, emit
from overrides import overrides

from snapcraft import pack
from snapcraft.parts import lifecycle as parts_lifecycle

if TYPE_CHECKING:
    import argparse


class _LifecycleCommand(BaseCommand, abc.ABC):
    """Run lifecycle-related commands."""

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        # TODO: add arguments for all step commands and pack
        pass

    @overrides
    def run(self, parsed_args):
        """Run the command."""
        if not self.name:
            raise RuntimeError("command name not specified")

        emit.trace(f"lifecycle command: {self.name!r}, arguments: {parsed_args!r}")
        parts_lifecycle.run(self.name, parsed_args)


class _LifecycleStepCommand(_LifecycleCommand):
    """Run lifecycle step commands."""

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        super().fill_parser(parser)
        parser.add_argument(
            "parts", metavar="parts", type=str, nargs="*", help="Parts to process"
        )


class PullCommand(_LifecycleStepCommand):
    """Run the lifecycle up to the pull step."""

    name = "pull"
    help_msg = "Download or retrieve artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Download or retrieve artifacts defined for a part. If part names
        are specified only those parts will be pulled, otherwise all parts
        will be pulled.
        """
    )


class BuildCommand(_LifecycleStepCommand):
    """Run the lifecycle up to the build step."""

    name = "build"
    help_msg = "Build artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Build artifacts defined for a part. If part names are specified only
        those parts will be built, otherwise all parts will be built.
        """
    )


class StageCommand(_LifecycleStepCommand):
    """Run the lifecycle up to the stage step."""

    name = "stage"
    help_msg = "Stage built artifacts into a common staging area"
    overview = textwrap.dedent(
        """
        Stage built artifacts into a common staging area. If part names are
        specified only those parts will be staged. The default is to stage
        all parts.
        """
    )


class PrimeCommand(_LifecycleStepCommand):
    """Prepare the final payload for packing."""

    name = "prime"
    help_msg = "Prime artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Prepare the final payload to be packed as a snap, performing additional
        processing and adding metadata files. If part names are specified only
        those parts will be primed. The default is to prime all parts.
        """
    )


class PackCommand(_LifecycleCommand):
    """Prepare the final payload for packing."""

    name = "pack"
    help_msg = "Build artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Prepare the final payload to be packed as a snap. If part names are
        specified only those parts will be primed. The default is to prime
        all parts.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        """Add arguments specific to the pack command."""
        super().fill_parser(parser)
        parser.add_argument(
            "directory",
            metavar="directory",
            type=str,
            nargs="?",
            default=None,
            help="Directory to pack",
        )
        parser.add_argument(
            "-o",
            "--output",
            metavar="filename",
            type=str,
            help="Path to the resulting snap",
        )

    @overrides
    def run(self, parsed_args):
        """Run the command."""
        if parsed_args.directory:
            pack.pack_snap(parsed_args.directory, output=parsed_args.output)
        else:
            super().run(parsed_args)
