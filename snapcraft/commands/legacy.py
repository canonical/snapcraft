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

"""Snapcraft commands that refer to the legacy implementation."""

from __future__ import annotations

import textwrap
from typing import TYPE_CHECKING

from craft_application.commands import AppCommand
from typing_extensions import override

from snapcraft.legacy_cli import run_legacy
from snapcraft.store._legacy_account import set_legacy_env

if TYPE_CHECKING:
    import argparse


class LegacyAppCommand(AppCommand):
    """Legacy command runner."""

    @override
    def run(self, parsed_args: argparse.Namespace):
        # Setup env var for legacy credentials.
        set_legacy_env()

        run_legacy()


#########
# Store #
#########


class StoreLegacyPromoteCommand(LegacyAppCommand):
    """Command passthrough for the promote command."""

    name = "promote"
    help_msg = "Promote a build set from a channel"
    overview = textwrap.dedent(
        """
        A build set is a set of commonly-tagged revisions; the simplest
        form of a build set is a set of revisions released to a channel.

        Currently, only channels are supported to release from (<from-channel>)

        Prior to releasing, visual confirmation shall be required.

        The format for channels is ``[<track>/]<risk>[/<branch>]`` where

        - <track> is used to support long-term release channels. It is
          implicitly set to the default.
        - <risk> is mandatory and must be one of ``stable``, ``candidate``,
          ``beta`` or ``edge``.
        - <branch> is optional and dynamically creates a channel with a
          specific expiration date. Branches are specifically designed
          to support short-term hot fixes.
        """
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "snap_name",
            metavar="snap-name",
        )
        parser.add_argument(
            "--from-channel",
            metavar="from-channel",
            help="the channel to promote from",
            required=True,
        )
        parser.add_argument(
            "--to-channel",
            metavar="to-channel",
            help="the channel to promote to",
            required=True,
        )
        parser.add_argument(
            "--yes", action="store_true", help="do not prompt for confirmation"
        )


##############
# Assertions #
##############


class StoreLegacySignBuildCommand(LegacyAppCommand):
    """Command passthrough for the sign-build command."""

    name = "sign-build"
    help_msg = "Sign a built snap file and assert it using the developer's key"
    overview = textwrap.dedent(
        """
        Sign a specific build of a snap with a given key and upload the assertion
        to the Snap Store (unless --local)."""
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--key-name", metavar="key-name", help="key used to sign the assertion"
        )
        parser.add_argument(
            "--local",
            action="store_true",
            help="sign assertion, but do not upload to the Snap Store",
        )
        parser.add_argument(
            "snap_file",
            metavar="snap-file",
            type=str,
            help="Snap file to sign",
        )


class StoreLegacyValidateCommand(LegacyAppCommand):
    """Command passthrough for the validate command."""

    name = "validate"
    help_msg = "Validate a gated snap"
    overview = textwrap.dedent(
        """
        Each validation can be specified with either syntax:

        -  <snap-name>=<revision>
        -  <snap-id>=<revision>"""
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--key-name", metavar="key-name", help="key used to sign the assertion"
        )
        parser.add_argument("--revoke", action="store_true", help="revoke validations")
        parser.add_argument("snap_name", metavar="snap-name")
        parser.add_argument("validations", nargs="+")
