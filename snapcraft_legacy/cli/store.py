# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2021 Canonical Ltd
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

import operator
from textwrap import dedent
from typing import Dict, List, Union

import click
from tabulate import tabulate

from snapcraft_legacy import storeapi

from . import echo
from ._options import add_verbosity_options


@click.group()
def storecli():
    """Store commands"""


def _human_readable_acls(store_client: storeapi.StoreClient) -> str:
    acl = store_client.acl()
    snap_names = []
    snap_ids = acl["snap_ids"]

    if snap_ids is not None:
        try:
            for snap_id in snap_ids:
                snap_names.append(store_client.get_snap_name_for_id(snap_id))
        except TypeError:
            raise RuntimeError(f"invalid snap_ids: {snap_ids!r}")
        acl["snap_names"] = snap_names
    else:
        acl["snap_names"] = None

    human_readable_acl: Dict[str, Union[str, List[str], None]] = {
        "expires": str(acl["expires"])
    }

    for key in ("snap_names", "channels", "permissions"):
        human_readable_acl[key] = acl[key]
        if not acl[key]:
            human_readable_acl[key] = "No restriction"

    return dedent(
        """\
        snaps:       {snap_names}
        channels:    {channels}
        permissions: {permissions}
        expires:     {expires}
    """.format(**human_readable_acl)
    )


@storecli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.option(
    "--from-channel",
    metavar="<from-channel>",
    required=True,
    help="The channel to promote from.",
)
@click.option(
    "--to-channel",
    metavar="<to-channel>",
    required=True,
    help="The channel to promote to.",
)
@click.option("--yes", is_flag=True, help="Do not prompt for confirmation.")
@add_verbosity_options()
def promote(snap_name, from_channel, to_channel, yes, **kwargs):
    """Promote a build set from to a channel.

    A build set is a set of commonly tagged revisions, the most simple
    form of a build set is a set of revisions released to a channel.

    Currently, only channels are supported to release from (<from-channel>)

    Prior to releasing, visual confirmation shall be required.

    The format for channels is `[<track>/]<risk>[/<branch>]` where

    \b
        - <track> is used to have long term release channels. It is implicitly
          set to the default.
        - <risk> is mandatory and can be either `stable`, `candidate`, `beta`
          or `edge`.
        - <branch> is optional and dynamically creates a channel with a
          specific expiration date.

    \b
    Examples:
        snapcraft promote my-snap --from-channel candidate --to-channel stable
        snapcraft promote my-snap --from-channel lts/candidate --to-channel lts/stable
        snapcraft promote my-snap --from-channel stable/patch --to-channel stable
        snapcraft promote my-snap --from-channel experimental/stable --to-channel stable
    """
    echo.warning(
        "snapcraft promote does not have a stable CLI interface. Use with caution in scripts."
    )
    parsed_from_channel = storeapi.channels.Channel(from_channel)
    parsed_to_channel = storeapi.channels.Channel(to_channel)

    if parsed_from_channel == parsed_to_channel:
        raise click.BadOptionUsage(
            "--to-channel", "--from-channel and --to-channel cannot be the same."
        )

    store = storeapi.StoreClient()
    status_payload = store.get_snap_status(snap_name)

    snap_status = storeapi.status.SnapStatus(
        snap_name=snap_name, payload=status_payload
    )
    from_channel_set = snap_status.get_channel_set(parsed_from_channel)
    echo.info("Build set information for {!r}".format(parsed_from_channel))
    click.echo(
        tabulate(
            sorted(from_channel_set, key=operator.attrgetter("arch")),
            headers=["Arch", "Revision", "Version"],
            tablefmt="plain",
        )
    )
    if yes or echo.confirm(
        "Do you want to promote the current set to the {!r} channel?".format(
            parsed_to_channel
        )
    ):
        for c in from_channel_set:
            store.release(
                snap_name=snap_name,
                revision=str(c.revision),
                channels=[str(parsed_to_channel)],
            )
        echo.wrapped(
            f"Promotion from {parsed_from_channel} to {parsed_to_channel} complete"
        )
    else:
        echo.wrapped("Channel promotion cancelled")
