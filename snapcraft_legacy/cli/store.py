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

import json
import operator
import os
from datetime import date, timedelta
from textwrap import dedent
from typing import Dict, List, Union

import click
from tabulate import tabulate

import snapcraft_legacy
from snapcraft_legacy import storeapi
from snapcraft_legacy._store import StoreClientCLI
from snapcraft_legacy.storeapi import metrics as metrics_module

from . import echo
from ._metrics import convert_metrics_to_table


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
    """.format(
            **human_readable_acl
        )
    )


@storecli.command("upload-metadata")
@click.option(
    "--force",
    is_flag=True,
    help="Force metadata update to override any possible conflict",
)
@click.argument(
    "snap-file",
    metavar="<snap-file>",
    type=click.Path(exists=True, readable=True, resolve_path=True, dir_okay=False),
)
def upload_metadata(snap_file, force):
    """Upload metadata from <snap-file> to the store.

    The following information will be retrieved from <snap-file> and used
    to update the store:

    \b
    - summary
    - description
    - icon

    If --force is given, it will force the local metadata into the Store,
    ignoring any possible conflict.

    \b
    Examples:
        snapcraft upload-metadata my-snap_0.1_amd64.snap
        snapcraft upload-metadata my-snap_0.1_amd64.snap --force
    """
    click.echo("Uploading metadata from {!r}".format(os.path.basename(snap_file)))
    snapcraft_legacy.upload_metadata(snap_file, force)


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
def promote(snap_name, from_channel, to_channel, yes):
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
    elif (
        parsed_from_channel.risk == "edge"
        and parsed_from_channel.branch is None
        and yes
    ):
        raise click.BadOptionUsage(
            "--from-channel",
            "{!r} is not a valid set value for --from-channel when using --yes.".format(
                parsed_from_channel
            ),
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


@storecli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.argument("track_name", metavar="<track>")
def set_default_track(snap_name: str, track_name: str):
    """Set the default track for <snap-name> to <track>.

    The track must be a valid active track for this operation to be successful.
    """
    store_client_cli = StoreClientCLI()

    metadata = dict(default_track=track_name)
    store_client_cli.upload_metadata(snap_name=snap_name, metadata=metadata, force=True)

    echo.info(f"Default track for {snap_name!r} set to {track_name!r}.")


_YESTERDAY = str(date.today() - timedelta(days=1))


@storecli.command()
@click.argument("snap-name", metavar="<snap-name>", required=True)
@click.option(
    "--name",
    metavar="<metric name>",
    help="Metric name",
    type=click.Choice([x.value for x in metrics_module.MetricsNames]),
    required=True,
)
@click.option(
    "--start",
    metavar="<start date>",
    help="Date in format YYYY-MM-DD",
    required=True,
    default=_YESTERDAY,
)
@click.option(
    "--end",
    metavar="<end date>",
    help="Date in format YYYY-MM-DD",
    required=True,
    default=_YESTERDAY,
)
@click.option(
    "--format",
    metavar="<format>",
    help="Format for output",
    type=click.Choice(["table", "json"]),
    required=True,
)
def metrics(snap_name: str, name: str, start: str, end: str, format: str):
    """Get metrics for <snap-name>."""
    store = storeapi.StoreClient()
    account_info = store.get_account_information()

    try:
        snap_id = account_info["snaps"][storeapi.constants.DEFAULT_SERIES][snap_name][
            "snap-id"
        ]
    except KeyError:
        echo.exit_error(
            brief="No permissions for snap.",
            resolution="Ensure the snap name and credentials are correct.is correct and that the correct credentials are used.",
        )

    mf = metrics_module.MetricsFilter(
        snap_id=snap_id, metric_name=name, start=start, end=end
    )

    results = store.get_metrics(filters=[mf], snap_name=snap_name)

    # Sanity check to ensure that only one result is found (as we currently only
    # support one query at a time).
    if len(results.metrics) != 1:
        raise RuntimeError(f"Unexpected metric results from store: {results!r}")

    metric_results = results.metrics[0]

    if format == "json":
        output = json.dumps(metric_results.marshal(), indent=2, sort_keys=True)
        click.echo(output)
    elif format == "table":
        rows = convert_metrics_to_table(metric_results, transpose=True)
        output = tabulate(rows, tablefmt="plain")
        echo.echo_with_pager_if_needed(output)
