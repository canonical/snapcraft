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
import argparse
import os
import textwrap

from craft_cli import BaseCommand, emit
from overrides import overrides

from snapcraft import pack, utils
from snapcraft.parts import lifecycle as parts_lifecycle


class _LifecycleCommand(BaseCommand, abc.ABC):
    """Lifecycle-related commands."""

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            "--destructive-mode",
            action="store_true",
            help="Build in the current host",
        )
        group.add_argument(
            "--use-lxd",
            action="store_true",
            help="Use LXD to build",
        )
        parser.add_argument(
            "--debug",
            action="store_true",
            help="Shell into the environment if the build fails",
        )
        parser.add_argument(
            "--enable-manifest",
            action="store_true",
            default=utils.strtobool(os.getenv("SNAPCRAFT_BUILD_INFO", "n")),
            help="Generate snap manifest",
        )
        parser.add_argument(
            "--manifest-image-information",
            type=str,
            metavar="image-info",
            default=os.getenv("SNAPCRAFT_IMAGE_INFO"),
            help="Set snap manifest image-info",
        )
        parser.add_argument(
            "--bind-ssh",
            action="store_true",
            help="Bind ~/.ssh directory to local build instances",
        )
        parser.add_argument(
            "--build-for",
            type=str,
            metavar="architecture",
            default=os.getenv("SNAPCRAFT_BUILD_FOR"),
            help="Set target architecture to build for",
        )
        parser.add_argument(
            "--http-proxy",
            type=str,
            default=os.getenv("http_proxy"),
            help="Set http proxy",
        )
        parser.add_argument(
            "--https-proxy",
            type=str,
            default=os.getenv("https_proxy"),
            help="Set https proxy",
        )
        parser.add_argument(
            "--ua-token",
            type=str,
            metavar="ua-token",
            default=os.getenv("SNAPCRAFT_UA_TOKEN"),
            help="Configure build environment with ESM using specified UA token",
        )
        parser.add_argument(
            "--enable-experimental-ua-services",
            action="store_true",
            help="Allow selection of UA services to enable.",
        )

        # --enable-experimental-extensions is only available in legacy
        parser.add_argument(
            "--enable-experimental-extensions",
            action="store_true",
            help=argparse.SUPPRESS,
        )
        # --enable-developer-debug is only available in legacy
        parser.add_argument(
            "--enable-developer-debug",
            action="store_true",
            help=argparse.SUPPRESS,
        )
        # --enable-experimental-target-arch is only available in legacy
        parser.add_argument(
            "--enable-experimental-target-arch",
            action="store_true",
            help=argparse.SUPPRESS,
        )
        # --target-arch is only available in legacy
        parser.add_argument("--target-arch", help=argparse.SUPPRESS)
        # --provider is only available in legacy
        parser.add_argument("--provider", help=argparse.SUPPRESS)

    @overrides
    def run(self, parsed_args):
        """Run the command."""
        if not self.name:
            raise RuntimeError("command name not specified")

        emit.debug(f"lifecycle command: {self.name!r}, arguments: {parsed_args!r}")
        parts_lifecycle.run(self.name, parsed_args)


class _LifecycleStepCommand(_LifecycleCommand):
    """Lifecycle step commands."""

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        super().fill_parser(parser)
        parser.add_argument(
            "parts",
            metavar="part-name",
            type=str,
            nargs="*",
            help="Optional list of parts to process",
        )

        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            "--shell",
            action="store_true",
            help="Shell into the environment in lieu of the step to run.",
        )
        group.add_argument(
            "--shell-after",
            action="store_true",
            help="Shell into the environment after the step has run.",
        )


class PullCommand(_LifecycleStepCommand):
    """Pull parts."""

    name = "pull"
    help_msg = "Download or retrieve artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Download or retrieve artifacts defined for a part. If part names
        are specified, only those parts will be pulled; otherwise, all parts
        will be pulled.
        """
    )


class BuildCommand(_LifecycleStepCommand):
    """Build parts."""

    name = "build"
    help_msg = "Build artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Build artifacts defined for a part. If part names are specified, only
        those parts will be built; otherwise, all parts will be built.
        """
    )


class StageCommand(_LifecycleStepCommand):
    """Stage parts."""

    name = "stage"
    help_msg = "Stage built artifacts into a common staging area"
    overview = textwrap.dedent(
        """
        Stage built artifacts into a common staging area. If part names are
        specified, only those parts will be staged. By default, all parts
        will be staged.
        """
    )


class PrimeCommand(_LifecycleStepCommand):
    """Prime parts."""

    name = "prime"
    help_msg = "Prime artifacts defined for a part"
    overview = textwrap.dedent(
        """
        Prepare the final payload to be packed as a snap, performing additional
        processing and adding metadata files. If part names are specified, only
        those parts will be primed. By default, all parts will be primed.
        """
    )


class PackCommand(_LifecycleCommand):
    """Pack the final snap payload."""

    name = "pack"
    help_msg = "Create the snap package"
    overview = textwrap.dedent(
        """
        Process parts and create a snap file containing the project payload
        with the provided metadata. If a directory is specified, pack its
        contents instead.
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
            snap_filename = pack.pack_snap(
                parsed_args.directory, output=parsed_args.output
            )
            emit.message(f"Created snap package {snap_filename}")
        else:
            super().run(parsed_args)


class SnapCommand(_LifecycleCommand):
    """Legacy command to pack the final snap payload."""

    name = "snap"
    help_msg = "Create a snap"
    hidden = True
    overview = textwrap.dedent(
        """
        Process parts and create a snap file containing the project payload
        with the provided metadata. This command is deprecated in favour
        of the newer 'pack' command.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        """Add arguments specific to the pack command."""
        super().fill_parser(parser)
        parser.add_argument(
            "-o",
            "--output",
            metavar="filename",
            type=str,
            help="Path to the resulting snap",
        )


class CleanCommand(_LifecycleStepCommand):
    """Remove part assets."""

    name = "clean"
    help_msg = "Remove a part's assets"
    overview = textwrap.dedent(
        """
        Clean up artifacts belonging to parts. If no parts are specified,
        remove the managed snap packing environment (VM or container).
        """
    )


class TryCommand(_LifecycleCommand):
    """Prepare the parts for ``snap try``."""

    name = "try"
    help_msg = 'Prepare a snap for "snap try".'
    overview = textwrap.dedent(
        """
        Process parts and expose the ``prime`` directory containing the
        final payload, in preparation for ``snap try prime``.
        """
    )

    @overrides
    def run(self, parsed_args):
        """Overridden to give a helpful message when the lifecycle finishes."""
        super().run(parsed_args)
        if not utils.is_managed_mode():
            emit.message("You can now run `snap try prime`")
