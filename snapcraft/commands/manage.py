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

"""Snapcraft Store Account management commands."""

from __future__ import annotations

import operator
import textwrap
from typing import TYPE_CHECKING, cast

from craft_application.commands import AppCommand
from craft_cli import emit
from craft_cli.errors import ArgumentParsingError
from tabulate import tabulate
from typing_extensions import override

from snapcraft import store, utils

if TYPE_CHECKING:
    import argparse


class StoreReleaseCommand(AppCommand):
    """Release a snap in the Snap Store."""

    name = "release"
    help_msg = "Release <snap-name> to the store"
    overview = textwrap.dedent(
        """
        Release <snap-name> at <revision> to the selected store <channels>.
        <channels> is a comma-separated list of valid channels in the store.

        The <revision> must exist in the store; to see available revisions,
        run ``snapcraft revisions <name>``.

        The channel map will be displayed after the operation takes place. To see
        the status map at any other time run ``snapcraft status <name>``.

        The format for a channel is ``[<track>/]<risk>[/<branch>]`` where

            - <track> is used to have long-term release channels. It is implicitly
              set to ``latest``. If this snap requires one, it can be created by
              request by having a conversation on https://forum.snapcraft.io
              under the *store* category.
            - <risk> is mandatory and must be one of ``stable``, ``candidate``, ``beta``
              or ``edge``.
            - <branch> is optional and dynamically creates a channel with a
              specific expiration date.

        Examples::

            snapcraft release my-snap 8 stable
            snapcraft release my-snap 8 stable/my-branch
            snapcraft release my-snap 9 beta,edge
            snapcraft release my-snap 9 lts-channel/stable
            snapcraft release my-snap 9 lts-channel/stable/my-branch"""
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "name",
            type=str,
            help="The snap name to release",
        )
        parser.add_argument(
            "revision",
            type=int,
            help="The revision to release",
        )
        parser.add_argument(
            "channels",
            type=str,
            help="The comma-separated list of channels to release to",
        )
        parser.add_argument(
            "--progressive",
            dest="progressive_percentage",
            type=int,
            default=None,
            help="set a release progression to a certain percentage [0<=x<=100]",
        )

    @override
    def run(self, parsed_args: argparse.Namespace):
        channels = parsed_args.channels.split(",")

        store.StoreClientCLI().release(
            snap_name=parsed_args.name,
            revision=parsed_args.revision,
            channels=channels,
            progressive_percentage=parsed_args.progressive_percentage,
        )

        humanized_channels = utils.humanize_list(channels, conjunction="and")
        progressive = parsed_args.progressive_percentage
        progressive_suffix = (
            f" for {progressive}% of users" if progressive is not None else ""
        )
        emit.message(
            f"Released {parsed_args.name!r} "
            f"revision {parsed_args.revision!r} "
            f"to channels: {humanized_channels}"
            f"{progressive_suffix}"
        )


class StorePromoteCommand(AppCommand):
    """Promote a build set from a channel in the Snap Store."""

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

    @override
    def run(self, parsed_args: argparse.Namespace):
        emit.warning(
            "snapcraft promote does not have a stable CLI interface. Use with caution in scripts."
        )
        from_channel = store.Channel(parsed_args.from_channel)
        to_channel = store.Channel(parsed_args.to_channel)

        if from_channel == to_channel:
            raise ArgumentParsingError(
                "--from-channel and --to-channel cannot be the same."
            )

        client = store.StoreClientCLI()
        status_payload = client.get_snap_status(parsed_args.snap_name)

        snap_status = store.SnapStatus(
            snap_name=parsed_args.snap_name, payload=status_payload
        )
        from_channel_set = snap_status.get_channel_set(from_channel)

        emit.progress(f"Build set information for {from_channel!r}", permanent=True)
        emit.progress(
            tabulate(
                sorted(from_channel_set, key=operator.attrgetter("arch")),
                headers=["Arch", "Revision", "Version"],
                tablefmt="plain",
            ),
            permanent=True,
        )

        if parsed_args.yes or emit.confirm(
            f"Do you want to promote the current set to the {to_channel!r} channel?"
        ):
            for c in from_channel_set:
                client.release(
                    snap_name=parsed_args.snap_name,
                    revision=cast(
                        int, c.revision
                    ),  # get_channel_set ensures this will not be none
                    channels=[str(to_channel)],
                )
            emit.message(f"Promotion from {from_channel} to {to_channel} complete")
        else:
            emit.message("Channel promotion cancelled")


class StoreCloseCommand(AppCommand):
    """Close a channel for a snap in the Snap Store."""

    name = "close"
    help_msg = "Close <channel> for <snap-name> in the store"
    overview = textwrap.dedent(
        """
        Closing a channel allows the channel that is closed to track the
        channel that follows it in the channel release chain.
        As such, closing the 'candidate' channel for a snap would make
        that snap begin to track the 'stable' channel.

        Examples::

            snapcraft close my-snap beta
        """
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "name",
            type=str,
            help="The snap name to release",
        )
        parser.add_argument(
            "channel",
            type=str,
            help="The channel to close",
        )

    @override
    def run(self, parsed_args: argparse.Namespace):
        client = store.StoreClientCLI()

        client.close(
            snap_name=parsed_args.name,
            channel=parsed_args.channel,
        )

        emit.message(
            f"Channel {parsed_args.channel!r} for {parsed_args.name!r} is now closed"
        )


class StoreSetDefaultTrackCommand(AppCommand):
    """Set the default track for a snap."""

    name = "set-default-track"
    help_msg = "Set the default track for a snap"
    overview = textwrap.dedent(
        """
        Set the default track for <snap-name> to <track>;
        the <track> must already exist.
        """
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "snap_name",
            metavar="snap-name",
        )
        parser.add_argument("track")

    @override
    def run(self, parsed_args: argparse.Namespace):
        client = store.StoreClientCLI()

        client.upload_metadata(
            snap_name=parsed_args.snap_name,
            metadata={"default_track": parsed_args.track},
            force=True,
        )

        emit.message(
            f"Default track for {parsed_args.snap_name!r} set to {parsed_args.track!r}."
        )
