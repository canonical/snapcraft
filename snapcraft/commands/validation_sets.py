# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Snapcraft Store Validation Sets commands."""

import json
import os
import subprocess
import tempfile
import textwrap
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Optional

import craft_application.util
import yaml
from craft_application.commands import AppCommand
from craft_application.errors import CraftValidationError
from craft_cli import emit
from overrides import overrides

from snapcraft import errors, utils
from snapcraft_legacy._store import StoreClientCLI
from snapcraft_legacy.storeapi.errors import StoreValidationSetsError
from snapcraft_legacy.storeapi.v2 import validation_sets

if TYPE_CHECKING:
    import argparse


_VALIDATIONS_SETS_SNAPS_TEMPLATE = textwrap.dedent(
    """\
    snaps:
      - name: hello  # The name of the snap.
    #    id:   <id>    # The ID of the snap. Optional, defaults to the current ID for
                       # the provided name.
    #    presence: [required|optional|invalid]  # Optional, defaults to required.
    #    revision: <n> # The revision of the snap. Optional.
    #    components: # Constraints to apply to the snap's components. Optional.
    #      <component-name>: [required|optional|invalid] # Short form for specifying the component's presence.
    #      <component-name>: # Long form that allows specifying the component's revision.
    #        presence: [required|optional|invalid] # Presence of the component. Required.
    #        revision: <n> # The revision of the component, required if the snap's revision is given.
                           # Otherwise, not allowed.
"""
)

_VALIDATION_SETS_TEMPLATE = textwrap.dedent(
    """\
    account-id: {account_id}
    name: {set_name}
    sequence: {sequence}
    # The revision for this validation set
    # revision: {revision}
    {snaps}
    """
)


class StoreEditValidationSetsCommand(AppCommand):
    """Edit a validation set."""

    name = "edit-validation-sets"
    help_msg = "Edit the list of validations for <snap-name>"
    overview = textwrap.dedent(
        """
        Refer to https://snapcraft.io/docs/validation-sets for further information
        on Validation Sets.
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--key-name", metavar="key-name", help="Key used to sign the assertion"
        )
        parser.add_argument("account_id", metavar="account-id")
        parser.add_argument("set_name", metavar="set-name")
        parser.add_argument("sequence", metavar="sequence")

    @overrides
    def run(self, parsed_args: "argparse.Namespace"):
        store_client = StoreClientCLI()

        asserted_validation_sets = store_client.get_validation_sets(
            name=parsed_args.set_name, sequence=str(parsed_args.sequence)
        )

        validation_sets_template = _generate_template(
            asserted_validation_sets,
            account_id=parsed_args.account_id,
            set_name=parsed_args.set_name,
            sequence=parsed_args.sequence,
        )

        with tempfile.NamedTemporaryFile() as temp_file:
            validation_sets_path = Path(temp_file.name)

        validation_sets_path.write_text(validation_sets_template, encoding="utf-8")
        edited_validation_sets = edit_validation_sets(validation_sets_path)

        if edited_validation_sets == validation_sets.EditableBuildAssertion(
            **yaml.safe_load(validation_sets_template)
        ):
            emit.message("No changes made")
            return

        try:
            while True:
                try:
                    _submit_validation_set(
                        edited_validation_sets, parsed_args.key_name, store_client
                    )
                    break
                except StoreValidationSetsError as validation_error:
                    emit.message(str(validation_error))
                    if not utils.confirm_with_user(
                        "Do you wish to amend the validation set?"
                    ):
                        raise errors.SnapcraftError(
                            "operation aborted"
                        ) from validation_error
                    edited_validation_sets = edit_validation_sets(validation_sets_path)
        finally:
            validation_sets_path.unlink()


def _submit_validation_set(
    edited_validation_sets: validation_sets.EditableBuildAssertion,
    key_name: Optional[str],
    store_client: StoreClientCLI,
) -> None:
    emit.debug(
        f"Posting assertion to build: {edited_validation_sets.model_dump_json()}"
    )
    build_assertion = store_client.post_validation_sets_build_assertion(
        validation_sets=edited_validation_sets.marshal()
    )
    build_assertion_dict = build_assertion.marshal_scalars_as_strings()

    signed_validation_sets = _sign_assertion(build_assertion_dict, key_name=key_name)

    emit.debug("Posting signed validation sets.")
    response = store_client.post_validation_sets(
        signed_validation_sets=signed_validation_sets
    )
    emit.debug(f"Response: {response.model_dump_json()}")


def _generate_template(
    asserted_validation_sets: validation_sets.ValidationSets,
    *,
    account_id: str,
    set_name: str,
    sequence: str,
) -> str:
    """Generate a template to edit asserted_validation_sets."""
    try:
        # assertions should only have one item since a specific
        # sequence was requested.
        revision = asserted_validation_sets.assertions[0].headers.revision

        snaps = yaml.dump(
            {
                "snaps": [
                    s.marshal()
                    for s in asserted_validation_sets.assertions[0].headers.snaps
                ]
            },
            default_flow_style=False,
            allow_unicode=True,
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

    return unverified_validation_sets


def edit_validation_sets(
    validation_sets_path: Path,
) -> validation_sets.EditableBuildAssertion:
    """Spawn an editor to modify the validation-sets."""
    editor_cmd = os.getenv("EDITOR", "vi")

    while True:
        with emit.pause():
            subprocess.run([editor_cmd, validation_sets_path], check=True)
        try:
            with validation_sets_path.open() as file:
                data = craft_application.util.safe_yaml_load(file)
            edited_validation_sets = (
                validation_sets.EditableBuildAssertion.from_yaml_data(
                    data=data,
                    # filepath is only shown for pydantic errors and snapcraft should
                    # not expose the temp file name
                    filepath=Path("validation-sets"),
                )
            )
            return edited_validation_sets
        except (yaml.YAMLError, CraftValidationError) as err:
            emit.message(f"{err!s}")
            if not utils.confirm_with_user("Do you wish to amend the validation set?"):
                raise errors.SnapcraftError("operation aborted") from err


def _sign_assertion(assertion: Dict[str, Any], *, key_name: Optional[str]) -> bytes:
    emit.debug("Signing assertion.")
    cmdline = ["snap", "sign"]
    if key_name:
        cmdline += ["-k", key_name]
    snap_sign = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    signed_assertion, _ = snap_sign.communicate(input=json.dumps(assertion).encode())
    if snap_sign.returncode != 0:
        raise errors.SnapcraftError("failed to sign assertion")

    return signed_assertion
