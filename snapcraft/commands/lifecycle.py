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

from craft_application.commands.lifecycle import PackCommand, LifecyclePartsCommand
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
        parser.add_argument(
            "--enable-experimental-plugins",
            action="store_true",
            default=os.getenv("SNAPCRAFT_ENABLE_EXPERIMENTAL_PLUGINS", "") != "",
            help="Allow using experimental (unstable) plugins.",
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


class SnapCommand(PackCommand):
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
