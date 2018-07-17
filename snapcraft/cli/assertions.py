# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import subprocess
import tempfile
from datetime import datetime
from textwrap import dedent
from typing import List, Dict

import click
import yaml

import snapcraft
from . import echo
from snapcraft import storeapi
from snapcraft.storeapi import assertions


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


@click.group()
def assertionscli():
    """Store assertion commands"""
    pass


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
def register_key(key_name: str) -> None:
    """Register a key with the store to sign assertions."""
    snapcraft.register_key(key_name)


@assertionscli.command("sign-build")
@click.option("--key-name", metavar="<key-name>")
@click.argument(
    "snap-file",
    metavar="<snap-file>",
    type=click.Path(exists=True, readable=True, resolve_path=True, dir_okay=False),
)
@click.option(
    "--local", is_flag=True, help="Do not push the generated assertion to the store"
)
def sign_build(snap_file: str, key_name: str, local: bool) -> None:
    """Sign a built snap file and assert it using the developer's key."""
    snapcraft.sign_build(snap_file, key_name=key_name, local=local)


@assertionscli.command()
@click.argument("snap-name", metavar="<snap-name>")
@click.argument("validations", metavar="<validation>...", nargs=-1, required=True)
@click.option("--key-name", metavar="<key-name>")
def validate(snap_name: str, validations: list, key_name: str) -> None:
    """Validate a gated snap."""
    snapcraft.validate(snap_name, validations, key=key_name)


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
        if click.confirm("Are you sure you want to continue?"):
            new_dev_assertion.push(force=True)
        else:
            echo.warning("The collaborators for this snap have not been altered.")


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
            yaml.dump(developer_wrapper, stream=fw, default_flow_style=False)
        subprocess.check_call([editor_cmd, ft.name])
        with open(ft.name, "r") as fr:
            developers = yaml.safe_load(fr).get("developers")
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
