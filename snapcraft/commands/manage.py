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

import textwrap
from typing import TYPE_CHECKING

from craft_cli import BaseCommand, emit
from overrides import overrides

from snapcraft import errors, store, utils

if TYPE_CHECKING:
    import argparse


class StoreReleaseCommand(BaseCommand):
    """Command to release a snap on the Snap Store."""

    name = "release"
    help_msg = "Release <snap-name> to the store"
    overview = textwrap.dedent(
        """
        Release <snap-name> on <revision> to the selected store <channels>.
        <channels> is a comma separated list of valid channels on the store.

        The <revision> must exist on the store, to see available revisions run
        `snapcraft list-revisions <snap_name>`.

        The channel map will be displayed after the operation takes place. To see
        the status map at any other time run `snapcraft status <snap-name>`.

        The format for channels is `[<track>/]<risk>[/<branch>]` where

            - <track> is used to have long term release channels. It is implicitly
              set to `latest`. If this snap requires one, it can be created by
              request by having a conversation on https://forum.snapcraft.io
              under the *store* category.
            - <risk> is mandatory and can be either `stable`, `candidate`, `beta`
              or `edge`.
            - <branch> is optional and dynamically creates a channel with a
              specific expiration date.

        Examples:
            snapcraft release my-snap 8 stable
            snapcraft release my-snap 8 stable/my-branch
            snapcraft release my-snap 9 beta,edge
            snapcraft release my-snap 9 lts-channel/stable
            snapcraft release my-snap 9 lts-channel/stable/my-branch"""
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
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
            help="The comma separated list of channels to release to",
        )
        parser.add_argument(
            "--progressive",
            dest="progressive_percentage",
            type=int,
            default=None,
            help="set a release progression to a certain percentage [0<=x<=100]",
        )

    @overrides
    def run(self, parsed_args):
        channels = parsed_args.channels.split(",")

        store.StoreClientCLI().release(
            snap_name=parsed_args.name,
            revision=parsed_args.revision,
            channels=channels,
            progressive_percentage=parsed_args.progressive_percentage,
        )

        humanized_channels = utils.humanize_list(channels, conjunction="and")
        emit.message(
            f"Released {parsed_args.name!r} "
            f"revision {parsed_args.revision!r} "
            f"to channels: {humanized_channels}"
        )


class StoreCloseCommand(BaseCommand):
    """Command to close a channel for a snap on the Snap Store."""

    name = "close"
    help_msg = "Close <channel> for <name> on the store"
    overview = textwrap.dedent(
        """
        Closing a channel allows the <channel> that is closed to track the
        channel that follows it in the channel release chain.
        As such closing the 'candidate' channel would make it track the
        'stable' channel.

        Examples:
            snapcraft close my-snap --channel beta
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
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

    @overrides
    def run(self, parsed_args):
        client = store.StoreClientCLI()

        # Account info request to retrieve the snap-id
        account_info = client.get_account_info()
        try:
            snap_id = account_info["snaps"][store.constants.DEFAULT_SERIES][
                parsed_args.name
            ]["snap-id"]
        except KeyError as key_error:
            emit.debug(f"{key_error!r} no found in {account_info!r}")
            raise errors.SnapcraftError(
                f"{parsed_args.name!r} not found or not owned by this account"
            ) from key_error

        client.close(
            snap_id=snap_id,
            channel=parsed_args.channel,
        )

        emit.message(
            f"Channel {parsed_args.channel!r} for {parsed_args.name!r} is now closed"
        )
