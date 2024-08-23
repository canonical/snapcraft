# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2024 Canonical Ltd.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License version 3, as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
#  SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Service class for registry set assertions."""

import json
import os
import pathlib
import subprocess
import tempfile
import textwrap
from typing import Any

import craft_application.util
import craft_cli
import tabulate
import yaml
from craft_application.errors import CraftValidationError

import snapcraft.utils

from .. import errors, store
from ..models import assertions
from . import AssertionService

_REGISTRY_SETS_TEMPLATE = textwrap.dedent(
    """\
    account-id: {account_id}
    name: {set_name}
    # summary: {summary}
    # The revision for this validation set
    # revision: {revision}
    views:
      {views}
    {body}
    """
)


_REGISTRY_SETS_VIEWS_TEMPLATE = textwrap.dedent(
    """\
    views:
      wifi-setup:
        rules:
          - request: ssids
            storage: wifi.ssids
            access: read
    """
)


_REGISTRY_SETS_BODY_TEMPLATE = textwrap.dedent(
    """\
    body:
      storage:
        schema:
          wifi:
            values:
              any
    """
)

class RegistrySetsService(AssertionService):

    @staticmethod
    def list_registry_sets(name: str | None) -> None:
        """Get a registry set from the store."""
        store_client = store.StoreClientCLI()
        payload = store_client.list_registry_sets(name)

        if payload.get("assertions"):
            registry_sets: list[tuple[str, str, str, str]] = []
            for item in payload.get("assertions"):
                registry = item["headers"]
                craft_cli.emit.debug(
                    f"parsing registry set: {json.dumps(registry, indent=2)}"
                )
                registry_sets.append(
                    (
                        registry["account-id"],
                        registry["name"],
                        registry["revision"],
                        registry["timestamp"][:10],
                    )
                )

            tabulated_sets = tabulate.tabulate(
                registry_sets,
                headers = ["Account ID", "Name", "Revision", "When"],
                tablefmt="plain",
            )
            craft_cli.emit.message(tabulated_sets)
        else:
            craft_cli.emit.message("No registry sets found.")

    @staticmethod
    def edit_registry_sets(name: str | None) -> None:
        """Edit a registry set."""
        store_client = store.StoreClientCLI()

        if name:
            registry_set_data = store_client.list_registry_sets(name)["assertions"][0]["headers"]
            incoming_registry_set = assertions.RegistryAssertion.unmarshal(registry_set_data)
        else:
            incoming_registry_set = None

        template = _generate_template(model=incoming_registry_set)
        craft_cli.emit.debug(f"template:\n  {template}")

        with tempfile.NamedTemporaryFile() as temp_file:
            registry_sets_path = pathlib.Path(temp_file.name)

        registry_sets_path.write_text(template, encoding="utf-8")
        editable_registry_set = _edit_registry_sets_user(registry_sets_path)

        craft_cli.emit.debug(f"registry set post-edit: {json.dumps(editable_registry_set.marshal(), indent=2)}")
        encoded_registry_set = _encode_assertion_as_dict(editable_registry_set)
        craft_cli.emit.debug(f"encoded registry set: {json.dumps(encoded_registry_set, indent=2)}")

        craft_cli.emit.progress("Building registry set.", permanent=True)
        registry_set_data = store_client.build_registry_assertion(encoded_registry_set)
        craft_cli.emit.debug(f"built registry set: {json.dumps(registry_set_data, indent=2)}")

        registry_set = assertions.RegistryAssertion.unmarshal(registry_set_data)
        registry_set.revision = incoming_registry_set.revision + 1
        registry_set_bytes = _encode_assertion_as_bytes(registry_set)
        signed_assertion = _sign_assertion(registry_set_bytes, key_name = "test6")

        craft_cli.emit.progress("Uploading registry set.")
        final_assertion = store_client.post_registry_sets(signed_assertion)

        craft_cli.emit.debug(f"{final_assertion=}")
        craft_cli.emit.progress("Registry set uploaded.", permanent=True)


def _generate_template(*, model: assertions.RegistryAssertion | None) -> str:
    if model:
        views = yaml.dump(
            {view: view_data.marshal() for view, view_data in model.views.items()},
            default_flow_style=False,
            allow_unicode=True,
        )
        registry_set_string = _REGISTRY_SETS_TEMPLATE.format(
            account_id=model.account_id,
            set_name=model.name,
            summary=model.summary,
            revision=model.revision,
            views=views,
            # XXX: the body is not returned when listing registry sets
            body=_REGISTRY_SETS_BODY_TEMPLATE,
        )
    else:
        registry_set_string = _REGISTRY_SETS_TEMPLATE.format(
            account_id="test-account-id",
            set_name="test-name",
            summary="test-summary",
            revision="0",
            views=_REGISTRY_SETS_VIEWS_TEMPLATE,
            body=_REGISTRY_SETS_BODY_TEMPLATE,
        )

    return registry_set_string

    # TODO: default_flow_style=True
    #return model.to_yaml_string()


def _encode_assertion_as_dict(model: assertions.Assertion) -> dict[str, Any]:
    """Encode the assertion for building.

    The `body` field needs to be a string of json data with 2 spaces of indentation
    and newlines.  For example, a model with:

      {"body": {"storage": {"schema": {"wifi": {"values": {"any"}}}}}}

    is encoded as:

      {\n  "storage": {\n    "schema": {\n      "wifi": {\n        "values":
      [\n          "any"\n        ]\n      }\n    }\n  }\n}
    """
    stringified_model = model.marshal()
    stringified_model["body"] = json.dumps(stringified_model["body"], indent=2)
    return stringified_model


def _encode_assertion_as_bytes(model: assertions.Assertion) -> bytes:
    """Encode the assertion for signing."""
    stringified_model = model.marshal_scalars_as_strings()
    json_data = json.dumps(stringified_model, indent=2)
    return json_data.encode()


def _sign_assertion(assertion: bytes, *, key_name: str | None) -> bytes:
    """Sign the assertion.

    :param assertion: A bytes object containing the assertion to sign.
    """
    craft_cli.emit.debug(f"Signing registry set: {assertion}")
    cmdline = ["snap", "sign"]
    if key_name:
        cmdline += ["-k", key_name]
    snap_sign = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    signed_assertion, _ = snap_sign.communicate(input=assertion)
    if snap_sign.returncode != 0:
        raise errors.SnapcraftError("failed to sign assertion")

    # print byte string with newlines
    craft_cli.emit.debug(f"signed assertion:\n{signed_assertion.decode()}")
    return signed_assertion


def _edit_registry_sets_user(registry_sets_path: pathlib.Path) -> assertions.EditableRegistryAssertion:
    """Spawn an editor to modify the validation-sets."""
    editor_cmd = os.getenv("EDITOR", "vi")

    while True:
        with craft_cli.emit.pause():
            subprocess.run([editor_cmd, registry_sets_path], check=True)
        try:
            with registry_sets_path.open() as file:
                data = craft_application.util.safe_yaml_load(file)
            edited_validation_sets = assertions.EditableRegistryAssertion.from_yaml_data(
                data=data,
                # filepath is only shown for pydantic errors and snapcraft should
                # not expose the temp file name
                filepath=pathlib.Path("registry-sets"),
            )
            return edited_validation_sets
        except (yaml.YAMLError, CraftValidationError) as err:
            craft_cli.emit.message(f"{err!s}")
            if not snapcraft.utils.confirm_with_user("Do you wish to amend the validation set?"):
                raise errors.SnapcraftError("operation aborted") from err
