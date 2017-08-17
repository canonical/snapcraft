# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import os
import sys
from textwrap import dedent

import click

import snapcraft
from snapcraft import storeapi
from snapcraft.storeapi.constants import DEFAULT_SERIES
from . import echo


_MESSAGE_REGISTER_PRIVATE = dedent("""\
    Even though this is private snap, you should think carefully about
    the choice of name and make sure you are confident nobody else will
    have a stronger claim to that particular name. If you are unsure
    then we suggest you prefix the name with your developer identity,
    As ‘nessita-yoyodyne-www-site-content’.""")
_MESSAGE_REGISTER_CONFIRM = dedent("""
    We always want to ensure that users get the software they expect
    for a particular name.

    If needed, we will rename snaps to ensure that a particular name
    reflects the software most widely expected by our community.

    For example, most people would expect ‘thunderbird’ to be published by
    Mozilla. They would also expect to be able to get other snaps of
    Thunderbird as 'thunderbird-$username'.

    Would you say that MOST users will expect {!r} to come from
    you, and be the software you intend to publish there?""")
_MESSAGE_REGISTER_SUCCESS = 'Congrats! You are now the publisher of {!r}.'
_MESSAGE_REGISTER_NO = dedent("""
    Thank you! {!r} will remain available.

    In the meantime you can register an alternative name.""")


@click.group()
def storecli():
    """Store commands"""
    pass


@storecli.command()
@click.argument('snap-name', metavar='<snap-name>')
@click.option('--private', is_flag=True,
              help='Register the snap as a private one')
def register(snap_name, private):
    """Register <snap-name> with the store.

    You can use this command to register an available <snap-name> and become
    the publisher for this snap.

    \b
    Examples:
        snapcraft register thunderbird
    """
    if private:
        click.echo(_MESSAGE_REGISTER_PRIVATE.format(snap_name))
    if click.confirm(_MESSAGE_REGISTER_CONFIRM.format(snap_name)):
        snapcraft.register(snap_name, private)
        click.echo(_MESSAGE_REGISTER_SUCCESS.format(snap_name))
    else:
        click.echo(_MESSAGE_REGISTER_NO.format(snap_name))


@storecli.command()
@click.option('--release', metavar='<channels>',
              help='Optional comma separated list of channels to release '
                   '<snap-file>')
@click.argument('snap-file', metavar='<snap-file>',
                type=click.Path(exists=True,
                                readable=True,
                                resolve_path=True,
                                dir_okay=False))
def push(snap_file, release):
    """Push <snap-file> to the store.
    By passing --release with a comma separated list of channels the snap would
    be released to the selected channels if the store review passes for this
    <snap-file>.

    This operation will block until the store finishes processing this
    <snap-file>.

    If --release is used, the channel map will be displayed after the
    operation takes place.

    \b
    Examples:
        snapcraft push my-snap_0.1_amd64.snap
        snapcraft push my-snap_0.2_amd64.snap --release edge
        snapcraft push my-snap_0.3_amd64.snap --release candidate,beta
    """
    click.echo('Pushing {}'.format(os.path.basename(snap_file)))
    channel_list = []
    if release:
        channel_list = release.split(',')
        click.echo(
            'After pushing, an attempt to release to {} '
            'will be made'.format(channel_list))

    snapcraft.push(snap_file, channel_list)


@storecli.command()
@click.argument('snap-name', metavar='<snap-name>')
@click.argument('revision', metavar='<revision>')
@click.argument('channels', metavar='<channels>')
def release(snap_name, revision, channels):
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
    snapcraft.release(snap_name, revision, channels.split(','))


@storecli.command()
@click.argument('snap-name', metavar='<snap-name>')
@click.argument('channels', metavar='<channel>...', nargs=-1)
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
    snapcraft.close(snap_name, channels)


@storecli.command()
@click.option('--arch', metavar='<arch>',
              help='The snap architecture to get the status for')
@click.option('--series', metavar='<series>',
              default=DEFAULT_SERIES,
              help='The snap series to get the status for')
@click.argument('snap-name', metavar='<snap-name>')
def status(snap_name, series, arch):
    """Get the status on the store for <snap-name>.

    \b
    Examples:
        snapcraft status my-snap
        snapcraft status my-snap --arch armhf
    """
    snapcraft.status(snap_name, series, arch)


@storecli.command('list-revisions')
@click.option('--arch', metavar='<arch>',
              help='The snap architecture to get the status for')
@click.option('--series', metavar='<series>',
              default=DEFAULT_SERIES,
              help='The snap series to get the status for')
@click.argument('snap-name', metavar='<snap-name>')
def list_revisions(snap_name, series, arch):
    """Get the history on the store for <snap-name>.

    This command has an alias of `revisions`.

    \b
    Examples:
        snapcraft list-revisions my-snap
        snapcraft list-revisions my-snap --arch armhf
        snapcraft revisions my-snap
    """
    snapcraft.revisions(snap_name, series, arch)


@storecli.command('list-registered')
def list_registered():
    """List snap names registered or shared with you.

    This command has an alias of `registered`.

    \b
    Examples:
        snapcraft list-registered
        snapcraft registered
    """
    snapcraft.list_registered()


@storecli.command()
def login():
    """Authenticate session against Ubuntu One SSO."""
    if not snapcraft.login():
        sys.exit(1)


@storecli.command()
def logout():
    """Clear session credentials."""
    echo.info('Clearing credentials for Ubuntu One SSO.')
    store = storeapi.StoreClient()
    store.logout()
    echo.info('Credentials cleared.')


@storecli.command()
def whoami():
    """Returns your login information relevant to the store."""
    try:
        account_data = storeapi.StoreClient().whoami()
    except storeapi.errors.InvalidCredentialsError:
        echo.error('You need to first login to use this command.')
        sys.exit(1)

    click.echo(dedent("""\
        email:        {email}
        developer-id: {account_id}""".format(**account_data)))

    # This is needed because we originally did not store the login information.
    if account_data['email'] == 'unknown':
        echo.warning('In order to view the correct email you will need to '
                     'logout and login again.')
