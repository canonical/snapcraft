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
import os
import json
from snapcraft.internal.errors import details_from_command_error
import subprocess
import tempfile
from datetime import datetime
from textwrap import dedent
from typing import Any, Dict, List

import click
from tabulate import tabulate

import snapcraft
from snapcraft._store import StoreClientCLI
from snapcraft import storeapi, yaml_utils
from snapcraft.storeapi import assertions

from . import echo

_COLLABORATION_HEADER = dedent(
    """\
    # Change which developers may push or release snaps on the publisher's behalf.
    #
    # Sample entry:
    #
    # developers:
    #   - developer-id: "dev-one"      # Which developer
    #     since: "2017-02-10 08:35:00" # When contributions started
    #     until: "2018-02-10 08:35:00" # When contributions ceased (optional)
    #
    # All timestamps are UTC, and the "now" special string will be replaced by
    # the current time. Do not remove entries or use an until time in the past
    # unless you want existing snaps provided by the developer to stop working."""
)  # noqa


_VALIDATIONS_SETS_SNAPS_TEMPLATE = dedent(
    """\
    snaps:
    #  - name: <name>  # The name of the snap.
    #    id:   <id>    # The ID of the snap. Optional, defaults to the current ID for
                       # the provided name.
    #    presence: [required|optional|invalid]  # Optional, defaults to required.
    #    revision: <n> # The revision of the snap. Optional.
"""
)

_VALIDATION_SETS_TEMPLATE = dedent(
    """\
    account-id: {account_id}
    name: {set_name}
    sequence: {sequence}
    # The revision for this validation set
    # revision: {revision}
    {snaps}
    """
)


@click.group()
def assertionscli():
    """Store assertion commands"""


@assertionscli.command("list-keys")
def list_keys():
    """List the keys available to sign assertions.

    This command has an alias of `keys`.
    """
    snapcraft.list_keys()


@assertionscli.command("create-key")
@click.argument("key-name", metavar="<key-name>", required=False)
def create_key(key_name: str) -> None:
    """Create a key to sign assertions."""
    snapcraft.create_key(key_name)


@assertionscli.command("register-key")
@click.argument("key-name", metavar="<key-name>", required=False)
@click.option(
    "--experimental-login",
    is_flag=True,
    help="*EXPERIMENTAL* Enables login through candid.",
    envvar="SNAPCRAFT_LOGIN",
)
def register_key(key_name: str, experimental_login: bool) -> None:
    """Register a key with the store to sign assertions."""
    snapcraft.register_key(key_name, use_candid=experimental_login)


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
def sign_build(snap_file: str, key_name: str, local: bool) -> None:
    """Sign a built snap file and assert it using the developer's key."""
    snapcraft.sign_build(snap_file, key_name=key_name, local=local)


@assertionscli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.argument("validations", metavar="<validation>...", nargs=-1, required=True)
@click.option("--key-name", metavar="<key-name>")
@click.option("--revoke/--no-revoke", default=False)
def validate(snap_name: str, validations: list, key_name: str, revoke: bool) -> None:
    """Validate a gated snap.

    Each validation can be presented with either syntax:

    -  <snap-name>=<revision>
    -  <snap-id>=<revision>
    """
    snapcraft.validate(snap_name, validations, revoke=revoke, key=key_name)


@assertionscli.command()
@click.argument("snap-name", metavar="<snap-name>")
def gated(snap_name: str) -> None:
    """Get the list of snaps and revisions gating a snap."""
    snapcraft.gated(snap_name)


@assertionscli.command("edit-collaborators")
@click.argument("snap-name", metavar="<snap-name>")
@click.option("--key-name", metavar="<key-name>")
def edit_collaborators(snap_name, key_name):
    """Edit the list of collaborators for <snap-name>.

    This command has an alias of `collaborators`.
    """
    dev_assertion = assertions.DeveloperAssertion(
        snap_name=snap_name, signing_key=key_name
    )
    developers = dev_assertion.get_developers()
    updated_developers = _update_developers(developers)
    new_dev_assertion = dev_assertion.new_assertion(developers=updated_developers)
    if new_dev_assertion.is_equal(dev_assertion):
        echo.warning("Aborting due to unchanged collaborators list.")
        return

    try:
        new_dev_assertion.push()
    except storeapi.errors.StoreValidationError as store_error:
        if store_error.error_list[0]["code"] != "revoked-uploads":
            raise store_error
        click.echo(
            "This will revoke the following collaborators: {!r}".format(
                " ".join(store_error.error_list[0]["extra"])
            )
        )
        if echo.confirm("Are you sure you want to continue?"):
            new_dev_assertion.push(force=True)
        else:
            echo.warning("The collaborators for this snap have not been altered.")


@assertionscli.command("list-validation-sets")
@click.option(
    "--name",
    metavar="<name>",
    help="Only show results for a given Validation Set name.",
)
@click.option("--sequence", metavar="<sequence>", help="Sequences to show.")
def list_validation_sets(name, sequence):
    """Get the list of validation sets.

    The sequence option can be a sequence number or a keyword.

    \b
    Examples:
        snapcraft list-validation-sets
        snapcraft list-validation-sets --sequence all
        snapcraft list-validation-sets --name my-set --sequence 1

    Refer to https://snapcraft.io/docs/validation-sets for further information
    on Validation Sets.
    """
    store_client = StoreClientCLI()
    asserted_validation_sets = store_client.get_validation_sets(
        name=name, sequence=sequence,
    )

    if not asserted_validation_sets.assertions and (name or sequence):
        echo.warning("No validation sets found for the requested name or sequence.")
    elif not asserted_validation_sets.assertions:
        echo.warning("No validation sets found for this account.")
    else:
        headers = ["Account-ID", "Name", "Sequence", "Revision", "When"]
        assertions = list()
        for assertion in asserted_validation_sets.assertions:
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


@assertionscli.command("edit-validation-sets")
@click.argument("account-id", metavar="<account-id>")
@click.argument("set-name", metavar="<set-name>")
@click.argument("sequence", metavar="<sequence>", type=int)
@click.option("--key-name", metavar="<key-name>")
def edit_validation_sets(account_id: str, set_name: str, sequence: int, key_name: str):
    """Edit the list of validations for <set-name>.

    Refer to https://snapcraft.io/docs/validation-sets for further information
    on Validation Sets.
    """
    store_client = StoreClientCLI()

    asserted_validation_sets = store_client.get_validation_sets(
        name=set_name, sequence=str(sequence)
    )

    try:
        # assertions should only have one item since a specific
        # sequence was requested.
        revision = asserted_validation_sets.assertions[0].revision
        snaps = yaml_utils.dump(
            {
                "snaps": [
                    s.marshal() for s in asserted_validation_sets.assertions[0].snaps
                ]
            }
        )
    except IndexError:
        # If there is no assertion for a given sequence, the store API
        # will return an empty list.
        revision = "0"
        snaps = _VALIDATIONS_SETS_SNAPS_TEMPLATE

    unverified_validation_sets = _VALIDATION_SETS_TEMPLATE.format(
        account_id=account_id,
        set_name=set_name,
        sequence=sequence,
        revision=revision,
        snaps=snaps,
    )

    edited_validation_sets = _edit_validation_sets(unverified_validation_sets)
    if edited_validation_sets == yaml_utils.load(unverified_validation_sets):
        echo.warning("No changes made.")
    else:
        build_assertion = store_client.post_validation_sets_build_assertion(
            validation_sets=edited_validation_sets
        )
        signed_validation_sets = _sign_assertion(
            build_assertion.marshal(), key_name=key_name
        )
        store_client.post_validation_sets(signed_validation_sets=signed_validation_sets)


def _edit_validation_sets(validation_sets: str) -> Dict[str, Any]:
    """Spawn an editor to modify the validation-sets."""
    editor_cmd = os.getenv("EDITOR", "vi")

    with tempfile.NamedTemporaryFile() as ft:
        ft.close()
        with open(ft.name, "w") as fw:
            print(validation_sets, file=fw)
        subprocess.run([editor_cmd, ft.name], check=True)
        with open(ft.name, "r") as fr:
            edited_validation_sets = yaml_utils.load(fr)

    return edited_validation_sets


def _sign_assertion(assertion: Dict[str, Any], *, key_name: str) -> bytes:
    cmdline = ["snap", "sign"]
    if key_name:
        cmdline += ["-k", key_name]
    snap_sign = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    signed_assertion, _ = snap_sign.communicate(input=json.dumps(assertion).encode())
    if snap_sign.returncode != 0:
        echo.exit_error(
            brief="Failed to sign assertion.",
            details=details_from_command_error(
                cmd=cmdline, returncode=snap_sign.returncode
            ),
            exit_code=snap_sign.returncode,
        )

    return signed_assertion


def _update_developers(developers: List[Dict[str, str]]) -> List[Dict[str, str]]:
    edit_friendly_developers = _reformat_time_for_editing(developers)
    updated_developers = _edit_developers(edit_friendly_developers)
    return _reformat_time_for_assertion(updated_developers)


def _edit_developers(developers: List[Dict[str, str]]) -> List[Dict[str, str]]:
    """Spawn an editor to modify the snap-developer assertion for a snap."""
    editor_cmd = os.getenv("EDITOR", "vi")

    developer_wrapper = {"developers": developers}

    with tempfile.NamedTemporaryFile() as ft:
        ft.close()
        with open(ft.name, "w") as fw:
            print(_COLLABORATION_HEADER, file=fw)
            yaml_utils.dump(developer_wrapper, stream=fw)
        subprocess.check_call([editor_cmd, ft.name])
        with open(ft.name, "r") as fr:
            developers = yaml_utils.load(fr).get("developers")
    return developers


def _reformat_time_for_editing(
    developers: List[Dict[str, str]], time_format: str = "%Y-%m-%d %H:%M:%S"
) -> List[Dict[str, str]]:
    reformatted_developers = []
    for developer in developers:
        developer_it = {"developer-id": developer["developer-id"]}
        for range in ["since", "until"]:
            if range in developer:
                date = datetime.strptime(developer[range], "%Y-%m-%dT%H:%M:%S.%fZ")
                developer_it[range] = datetime.strftime(date, time_format)
        reformatted_developers.append(developer_it)
    return reformatted_developers


def _reformat_time_for_assertion(
    developers: List[Dict[str, str]]
) -> List[Dict[str, str]]:
    reformatted_developers = []
    for developer in developers:
        developer_it = {"developer-id": developer["developer-id"]}
        for range_ in ["since", "until"]:
            if range_ in developer:
                if developer[range_] == "now":
                    date = datetime.now()
                else:
                    date = datetime.strptime(developer[range_], "%Y-%m-%d %H:%M:%S")
                # We don't care about microseconds because we cannot edit
                # later so we set that to 0.
                developer_it[range_] = date.strftime("%Y-%m-%dT%H:%M:%S.000000Z")
        reformatted_developers.append(developer_it)
    return reformatted_developers
