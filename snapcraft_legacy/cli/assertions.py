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
from datetime import datetime

import click
from tabulate import tabulate

import snapcraft_legacy
from snapcraft_legacy import storeapi
from snapcraft_legacy._store import StoreClientCLI

from . import echo
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


@assertionscli.command("list-validation-sets")
@click.option(
    "--name",
    metavar="<name>",
    help="Only show results for a given Validation Set name.",
)
@click.option("--sequence", metavar="<sequence>", help="Sequences to show.")
@add_verbosity_options()
def list_validation_sets(name, sequence, **kwargs):
    """Get the list of validation sets.

    The sequence option can be a sequence number or a keyword.

    \b
    Examples:
        snapcraft validation-sets
        snapcraft validation-sets --sequence all
        snapcraft validation-sets --name my-set --sequence 1

    Refer to https://snapcraft.io/docs/validation-sets for further information
    on Validation Sets.
    """
    store_client = StoreClientCLI()
    asserted_validation_sets = store_client.get_validation_sets(
        name=name,
        sequence=sequence,
    )

    if not asserted_validation_sets.assertions and (name or sequence):
        echo.warning("No validation sets found for the requested name or sequence.")
    elif not asserted_validation_sets.assertions:
        echo.warning("No validation sets found for this account.")
    else:
        headers = ["Account-ID", "Name", "Sequence", "Revision", "When"]
        assertions = list()
        for assertion_header in asserted_validation_sets.assertions:
            assertion = assertion_header.headers
            assertions.append(
                [
                    assertion.account_id,
                    assertion.name,
                    assertion.sequence,
                    assertion.revision,
                    datetime.strptime(
                        assertion.timestamp, "%Y-%m-%dT%H:%M:%SZ"
                    ).strftime("%Y-%m-%d"),
                ]
            )

        click.echo(
            tabulate(assertions, numalign="left", headers=headers, tablefmt="plain")
        )
