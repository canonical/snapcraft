# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2021 Canonical Ltd
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

import click

import snapcraft_legacy
from snapcraft_legacy import storeapi

from ._options import add_verbosity_options


@click.group()
def assertionscli():
    """Store assertion commands"""


@assertionscli.command("list-keys")
@add_verbosity_options()
def list_keys(**kwargs):
    """List the keys available to sign assertions.

    This command has an alias of `keys`.
    """
    snapcraft_legacy.list_keys()


@assertionscli.command("create-key")
@click.argument("key-name", metavar="<key-name>", required=False)
@add_verbosity_options()
def create_key(key_name: str, **kwargs) -> None:
    """Create a key to sign assertions."""
    snapcraft_legacy.create_key(key_name)


@assertionscli.command("register-key")
@click.argument("key-name", metavar="<key-name>", required=False)
@click.option(
    "--experimental-login",
    is_flag=True,
    help="*EXPERIMENTAL* Enables login through candid.",
    envvar="SNAPCRAFT_LOGIN",
)
@add_verbosity_options()
def register_key(key_name: str, experimental_login: bool, **kwargs) -> None:
    """Register a key with the store to sign assertions."""
    if experimental_login:
        raise click.BadArgumentUsage(
            f"Set {storeapi.constants.ENVIRONMENT_STORE_AUTH}=candid instead"
        )
    snapcraft_legacy.register_key(key_name)


@assertionscli.command("sign-build")
@click.option("--key-name", metavar="<key-name>")
@click.argument(
    "snap-file",
    metavar="<snap-file>",
    type=click.Path(exists=True, readable=True, resolve_path=True, dir_okay=False),
)
@click.option(
    "--local", is_flag=True, help="Do not upload the generated assertion to the store"
)
@add_verbosity_options()
def sign_build(snap_file: str, key_name: str, local: bool, **kwargs) -> None:
    """Sign a built snap file and assert it using the developer's key."""
    snapcraft_legacy.sign_build(snap_file, key_name=key_name, local=local)


@assertionscli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.argument("validations", metavar="<validation>...", nargs=-1, required=True)
@click.option("--key-name", metavar="<key-name>")
@click.option("--revoke/--no-revoke", default=False)
@add_verbosity_options()
def validate(
    snap_name: str, validations: list, key_name: str, revoke: bool, **kwargs
) -> None:
    """Validate a gated snap.

    Each validation can be presented with either syntax:

    -  <snap-name>=<revision>
    -  <snap-id>=<revision>
    """
    snapcraft_legacy.validate(snap_name, validations, revoke=revoke, key=key_name)


@assertionscli.command()
@click.argument("snap-name", metavar="<snap-name>")
@add_verbosity_options()
def gated(snap_name: str, **kwargs) -> None:
    """Get the list of snaps and revisions gating a snap."""
    snapcraft_legacy.gated(snap_name)
