# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2020 Canonical Ltd
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

import contextlib
import os
import functools
import operator
import stat
import sys
from textwrap import dedent
from typing import Dict, List, Optional, Union

import click
from tabulate import tabulate

import snapcraft
from snapcraft._store import StoreClientCLI
from snapcraft import storeapi, formatting_utils
from snapcraft.storeapi.constants import DEFAULT_SERIES
from . import echo
from ._review import review_snap
from ._channel_map import get_tabulated_channel_map


_MESSAGE_REGISTER_PRIVATE = dedent(
    """\
    Even though this is private snap, you should think carefully about
    the choice of name and make sure you are confident nobody else will
    have a stronger claim to that particular name. If you are unsure
    then we suggest you prefix the name with your developer identity,
    As '$username-yoyodyne-www-site-content'."""
)
_MESSAGE_REGISTER_CONFIRM = dedent(
    """
    We always want to ensure that users get the software they expect
    for a particular name.

    If needed, we will rename snaps to ensure that a particular name
    reflects the software most widely expected by our community.

    For example, most people would expect 'thunderbird' to be published by
    Mozilla. They would also expect to be able to get other snaps of
    Thunderbird as '$username-thunderbird'.

    Would you say that MOST users will expect {!r} to come from
    you, and be the software you intend to publish there?"""
)
_MESSAGE_REGISTER_SUCCESS = "Congrats! You are now the publisher of {!r}."
_MESSAGE_REGISTER_NO = dedent(
    """
    Thank you! {!r} will remain available.

    In the meantime you can register an alternative name."""
)


@contextlib.contextmanager
def _requires_login():
    try:
        yield
    except storeapi.errors.InvalidCredentialsError:
        echo.error("No valid credentials found." ' Have you run "snapcraft login"?')
        raise


@click.group()
def storecli():
    """Store commands"""
    pass


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


@storecli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.option("--private", is_flag=True, help="Register the snap as a private one")
@click.option("--store", metavar="<store>", help="Store to register with")
@click.option("--yes", is_flag=True)
def register(snap_name, private, store, yes):
    """Register <snap-name> with the store.

    You can use this command to register an available <snap-name> and become
    the publisher for this snap.

    \b
    Examples:
        snapcraft register thunderbird
    """
    if private:
        click.echo(_MESSAGE_REGISTER_PRIVATE.format(snap_name))
    if yes or echo.confirm(_MESSAGE_REGISTER_CONFIRM.format(snap_name)):
        snapcraft.register(snap_name, is_private=private, store_id=store)
        click.echo(_MESSAGE_REGISTER_SUCCESS.format(snap_name))
    else:
        click.echo(_MESSAGE_REGISTER_NO.format(snap_name))


@storecli.command()
@click.option(
    "--release",
    metavar="<channels>",
    help="Optional comma separated list of channels to release <snap-file>",
)
@click.argument(
    "snap-file",
    metavar="<snap-file>",
    type=click.Path(exists=True, readable=True, resolve_path=True, dir_okay=False),
)
def upload(snap_file, release):
    """Upload <snap-file> to the store.

    By passing --release with a comma separated list of channels the snap would
    be released to the selected channels if the store review passes for this
    <snap-file>.

    This operation will block until the store finishes processing this
    <snap-file>.

    If --release is used, the channel map will be displayed after the
    operation takes place.

    \b
    Examples:
        snapcraft upload my-snap_0.1_amd64.snap
        snapcraft upload my-snap_0.2_amd64.snap --release edge
        snapcraft upload my-snap_0.3_amd64.snap --release candidate,beta
    """
    click.echo("Preparing to upload {!r}.".format(os.path.basename(snap_file)))
    if release:
        channel_list = release.split(",")
        click.echo(
            "After uploading, the resulting snap revision will be released to "
            "{} when it passes the Snap Store review."
            "".format(formatting_utils.humanize_list(channel_list, "and"))
        )
    else:
        channel_list = None

    review_snap(snap_file=snap_file)
    snap_name, snap_revision = snapcraft.upload(snap_file, channel_list)

    echo.info("Revision {!r} of {!r} created.".format(snap_revision, snap_name))
    if channel_list:
        store_client_cli = StoreClientCLI()
        snap_channel_map = store_client_cli.get_snap_channel_map(snap_name=snap_name)

        click.echo(
            get_tabulated_channel_map(
                snap_channel_map,
                architectures=snap_channel_map.get_revision(
                    snap_revision
                ).architectures,
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
    snapcraft.upload_metadata(snap_file, force)


@storecli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.argument("revision", metavar="<revision>")
@click.argument("channels", metavar="<channels>")
@click.option(
    "--progressive",
    type=click.IntRange(0, 100),
    default=100,
    metavar="<percentage>",
    help="set a release progression to a certain percentage.",
)
@click.option(
    "--experimental-progressive-releases",
    is_flag=True,
    help="*EXPERIMENTAL* Enables 'progressive releases'.",
    envvar="SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES",
)
def release(
    snap_name,
    revision,
    channels,
    progressive: Optional[int],
    experimental_progressive_releases: bool,
) -> None:
    """Release <snap-name> on <revision> to the selected store <channels>.
    <channels> is a comma separated list of valid channels on the
    store.

    The <revision> must exist on the store, to see available revisions
    run `snapcraft list-revisions <snap_name>`.

    The channel map will be displayed after the operation takes place.
    To see the status map at any other time run `snapcraft status <snap-name>`.

    The format for channels is `[<track>/]<risk>[/<branch>]` where

    \b
        - <track> is used to have long term release channels. It is implicitly
          set to `latest`. If this snap requires one, it can be created by
          request by having a conversation on https://forum.snapcraft.io
          under the *store* category.
        - <risk> is mandatory and can be either `stable`, `candidate`, `beta`
          or `edge`.
        - <branch> is optional and dynamically creates a channel with a
          specific expiration date.

    \b
    Examples:
        snapcraft release my-snap 8 stable
        snapcraft release my-snap 8 stable/my-branch
        snapcraft release my-snap 9 beta,edge
        snapcraft release my-snap 9 lts-channel/stable
        snapcraft release my-snap 9 lts-channel/stable/my-branch
    """
    # If progressive is set to 100, treat it as None.
    if progressive == 100:
        progressive = None

    if progressive is not None and not experimental_progressive_releases:
        raise click.UsageError(
            "--progressive requires --experimental-progressive-releases."
        )
    elif progressive:
        os.environ["SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES"] = "Y"
        echo.warning("*EXPERIMENTAL* progressive releases in use.")

    store_client_cli = StoreClientCLI()
    release_data = store_client_cli.release(
        snap_name=snap_name,
        revision=revision,
        channels=channels.split(","),
        progressive_percentage=progressive,
    )
    snap_channel_map = store_client_cli.get_snap_channel_map(snap_name=snap_name)
    architectures_for_revision = snap_channel_map.get_revision(
        int(revision)
    ).architectures
    tracks = [storeapi.channels.Channel(c).track for c in channels.split(",")]
    click.echo(
        get_tabulated_channel_map(
            snap_channel_map, tracks=tracks, architectures=architectures_for_revision
        )
    )

    opened_channels = release_data.get("opened_channels", [])
    if len(opened_channels) == 1:
        echo.info(f"The {opened_channels[0]!r} channel is now open.")
    elif len(opened_channels) > 1:
        channels = ("{!r}".format(channel) for channel in opened_channels[:-1])
        echo.info(
            "The {} and {!r} channels are now open.".format(
                ", ".join(channels), opened_channels[-1]
            )
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
    with _requires_login():
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
        snap_channel_map = store.get_snap_channel_map(snap_name=snap_name)
        existing_architectures = snap_channel_map.get_existing_architectures()
        click.echo(
            get_tabulated_channel_map(
                snap_channel_map,
                tracks=[parsed_to_channel.track],
                architectures=existing_architectures,
            )
        )
    else:
        echo.wrapped("Channel promotion cancelled")


@storecli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.argument("channels", metavar="<channel>...", nargs=-1)
def close(snap_name, channels):
    """Close <channel> for <snap-name>.
    Closing a channel allows the <channel> that is closed to track the channel
    that follows it in the channel release chain. As such closing the
    'candidate' channel would make it track the 'stable' channel.

    The channel map will be displayed after the operation takes place.

    \b
    Examples:
        snapcraft close my-snap beta
        snapcraft close my-snap beta edge
    """
    store = storeapi.StoreClient()
    account_info = store.get_account_information()

    try:
        snap_id = account_info["snaps"][DEFAULT_SERIES][snap_name]["snap-id"]
    except KeyError:
        raise storeapi.errors.StoreChannelClosingPermissionError(
            snap_name, DEFAULT_SERIES
        )

    # Returned closed_channels cannot be trusted as it returns risks.
    store.close_channels(snap_id=snap_id, channel_names=channels)
    if len(channels) == 1:
        msg = "The {} channel is now closed.".format(channels[0])
    else:
        msg = "The {} and {} channels are now closed.".format(
            ", ".join(channels[:-1]), channels[-1]
        )

    snap_channel_map = store.get_snap_channel_map(snap_name=snap_name)
    if snap_channel_map.channel_map:
        closed_tracks = {storeapi.channels.Channel(c).track for c in channels}
        existing_architectures = snap_channel_map.get_existing_architectures()

        click.echo(
            get_tabulated_channel_map(
                snap_channel_map,
                architectures=existing_architectures,
                tracks=closed_tracks,
            )
        )
        click.echo()

    echo.info(msg)


@storecli.command()
@click.option(
    "--experimental-progressive-releases",
    is_flag=True,
    help="*EXPERIMENTAL* Enables 'progressive releases'.",
    envvar="SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES",
)
@click.option(
    "--arch", metavar="<arch>", help="The snap architecture to get the status for"
)
@click.argument("snap-name", metavar="<snap-name>")
def status(snap_name, arch, experimental_progressive_releases):
    """Get the status on the store for <snap-name>.

    \b
    Examples:
        snapcraft status my-snap
    """
    if experimental_progressive_releases:
        os.environ["SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES"] = "Y"
        echo.warning("*EXPERIMENTAL* progressive releases in use.")

    snap_channel_map = StoreClientCLI().get_snap_channel_map(snap_name=snap_name)
    existing_architectures = snap_channel_map.get_existing_architectures()

    if not snap_channel_map.channel_map:
        echo.warning("This snap has no released revisions.")
    elif arch and arch not in existing_architectures:
        echo.warning(f"No revisions for architecture {arch!r}.")
    else:
        if arch:
            architectures = (arch,)
        else:
            architectures = existing_architectures
        click.echo(
            get_tabulated_channel_map(snap_channel_map, architectures=architectures)
        )


@storecli.command("list-revisions")
@click.option(
    "--arch", metavar="<arch>", help="The snap architecture to get the status for"
)
@click.argument("snap-name", metavar="<snap-name>")
def list_revisions(snap_name, arch):
    """Get the history on the store for <snap-name>.

    This command has an alias of `revisions`.

    \b
    Examples:
        snapcraft list-revisions my-snap
        snapcraft list-revisions my-snap --arch armhf
        snapcraft revisions my-snap
    """
    snapcraft.revisions(snap_name, arch)


@storecli.command("list")
def list():
    """List snap names registered or shared with you.

    \b
    Examples:
        snapcraft list
    """
    snapcraft.list_registered()


@storecli.command("export-login")
@click.argument(
    "login_file", metavar="FILE", type=click.Path(dir_okay=False, writable=True)
)
@click.option(
    "--snaps", metavar="<snaps>", help="Comma-separated list of snaps to limit access"
)
@click.option(
    "--channels",
    metavar="<channels>",
    help="Comma-separated list of channels to limit access",
)
@click.option(
    "--acls", metavar="<acls>", help="Comma-separated list of ACLs to limit access"
)
@click.option(
    "--expires",
    metavar="<expiration date>",
    help="Date/time (in ISO 8601) when this exported login expires",
)
def export_login(login_file: str, snaps: str, channels: str, acls: str, expires: str):
    """Save login configuration for a store account in FILE.

    This file can then be used to log in to the given account with the
    specified permissions. One can also request the login to be exported to
    stdout instead of a file:

        snapcraft export-login -

    For example, to limit access to the edge channel of any snap the account
    can access:

        snapcraft export-login --channels=edge exported

    Or to limit access to only the edge channel of a single snap:

        snapcraft export-login --snaps=my-snap --channels=edge exported

    To limit access to a single snap, but only until 2019:

        snapcraft export-login --expires="2019-01-01T00:00:00" exported
    """

    snap_list = None
    channel_list = None
    acl_list = None

    if snaps:
        snap_list = []
        for package in snaps.split(","):
            snap_list.append({"name": package, "series": DEFAULT_SERIES})

    if channels:
        channel_list = channels.split(",")

    if acls:
        acl_list = acls.split(",")

    store_client = storeapi.StoreClient()
    if not snapcraft.login(
        store=store_client,
        packages=snap_list,
        channels=channel_list,
        acls=acl_list,
        expires=expires,
        save=False,
    ):
        sys.exit(1)

    # Support a login_file of '-', which indicates a desire to print to stdout
    if login_file.strip() == "-":
        echo.info("\nExported login starts on next line:")
        store_client.conf.save(config_fd=sys.stdout, encode=True)
        print()

        preamble = "Login successfully exported and printed above"
        login_action = 'echo "<login>" | snapcraft login --with -'
    else:
        # This is sensitive-- it should only be accessible by the owner
        private_open = functools.partial(os.open, mode=0o600)

        # mypy doesn't have the opener arg in its stub. Ignore its warning
        with open(login_file, "w", opener=private_open) as f:  # type: ignore
            store_client.conf.save(config_fd=f)

        # Now that the file has been written, we can just make it
        # owner-readable
        os.chmod(login_file, stat.S_IRUSR)

        preamble = "Login successfully exported to {0!r}".format(login_file)
        login_action = "snapcraft login --with {0}".format(login_file)

    print()
    echo.info(
        dedent(
            """\
        {}. This can now be used with

            {}

        to log in to this account with no password and have these
        capabilities:\n""".format(
                preamble, login_action
            )
        )
    )
    echo.info(_human_readable_acls(store_client))
    echo.warning(
        "This exported login is not encrypted. Do not commit it to version control!"
    )


@storecli.command()
@click.option(
    "--with",
    "login_file",
    metavar="<login file>",
    type=click.File("r"),
    help="Path to file created with 'snapcraft export-login'",
)
def login(login_file):
    """Login with your Ubuntu One e-mail address and password.

    If you do not have an Ubuntu One account, you can create one at
    https://snapcraft.io/account
    """
    store_client = storeapi.StoreClient()
    if not snapcraft.login(store=store_client, config_fd=login_file):
        sys.exit(1)

    print()

    if login_file:
        echo.info("Login successful. You now have these capabilities:\n")
        echo.info(_human_readable_acls(store_client))
    else:
        echo.info("Login successful.")


@storecli.command()
def logout():
    """Clear session credentials."""
    store = storeapi.StoreClient()
    store.logout()
    echo.info("Credentials cleared.")


@storecli.command()
def whoami():
    """Returns your login information relevant to the store."""
    try:
        account_data = storeapi.StoreClient().whoami()
    except storeapi.errors.InvalidCredentialsError:
        echo.error("You need to first login to use this command.")
        sys.exit(1)

    click.echo(
        dedent(
            """\
        email:        {email}
        developer-id: {account_id}""".format(
                **account_data
            )
        )
    )

    # This is needed because we originally did not store the login information.
    if account_data["email"] == "unknown":
        echo.warning(
            "In order to view the correct email you will need to "
            "logout and login again."
        )
