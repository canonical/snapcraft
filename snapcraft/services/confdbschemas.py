# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Service class for confdb schemas."""

from __future__ import annotations

import collections
import textwrap
from typing import Any, override

import yaml

from snapcraft import models
from snapcraft.services import Assertion

# Template parts to be assembled
_TPL_BASE = textwrap.dedent(
    """\
    account-id: {account_id}
    name: {name}"""
)
_TPL_REVISION_COMMENT = "# The revision for this confdb-schema\n# revision: {revision}"
# Body template - {body_block_manual} will be the manually constructed 'body: |' block
_TPL_BODY = "{body_block_manual}"

# Default view template for new schemas
_CONFDB_SCHEMA_VIEWS_TEMPLATE_DEFAULT = textwrap.dedent(
    """\
    views:
      wifi-setup:
        rules:
          - request: ssids
            storage: wifi.ssids
            access: read
    """
)

# Default body template for new schemas
_CONFDB_SCHEMA_BODY_TEMPLATE_DEFAULT = textwrap.dedent(
    """\
    body: |
      {
        "storage": {
          "schema": {
            "wifi": {
              "values": "any"
            }
          }
        }
      }
    """
)

# --- Key Ordering and Custom Dumper ---
# Define the desired order for keys within rules
RULE_KEY_ORDER = ["request", "storage", "access", "summary", "content"]
# Define the desired order for keys within views
VIEW_KEY_ORDER = ["summary", "rules"]
# Top level order (after base fields)
TOP_LEVEL_ORDER = ["summary", "views", "body"]

# Custom Dumper to preserve OrderedDict order explicitly with PyYAML
class OrderedDumper(yaml.SafeDumper):
    """A YAML Dumper that respects OrderedDict."""

OrderedDumper.add_representer(
    collections.OrderedDict,
    lambda dumper, data: dumper.represent_mapping(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, data.items()
    ),
)

def _dict_to_ordered_dict(data: Any, ordered_keys: list[str] | None = None) -> Any:
    """Recursively converts dicts to OrderedDicts, respecting key order."""
    if isinstance(data, dict):
        # Determine the key order for this level
        current_order = None
        if "request" in data: # It's a rule
            current_order = RULE_KEY_ORDER
        elif "rules" in data: # It's a view content dict
             current_order = VIEW_KEY_ORDER
        elif ordered_keys: # Use provided order if available
             current_order = ordered_keys
        # else: use insertion order (or default dict order for older Python)

        new_dict = collections.OrderedDict()
        processed_keys = set()

        if current_order:
            # Add keys in the specified order first
            for key in current_order:
                if key in data:
                    value = data[key]
                    # Skip empty summary
                    if key == "summary" and not value:
                        continue
                     # Recursively process the value
                    new_dict[key] = _dict_to_ordered_dict(value)
                    processed_keys.add(key)

        # Add any remaining keys not in the specified order
        for key, value in data.items():
             if key not in processed_keys:
                 # Skip empty summary
                 if key == "summary" and not value:
                     continue
                 new_dict[key] = _dict_to_ordered_dict(value)
        return new_dict
    if isinstance(data, list):
        # Process each item in the list recursively
        return [_dict_to_ordered_dict(item) for item in data]
    # Return non-dict/list items as is
    return data


class ConfdbSchemas(Assertion):
    """Service for interacting with confdb schemas."""

    @property
    @override
    def _assertion_name(self) -> str:
        return "confdb-schema"

    @property
    @override
    def _editable_assertion_class(self) -> type[models.EditableAssertion]:
        return models.EditableConfdbSchemaAssertion

    @override
    def _get_assertions(self, name: str | None = None) -> list[models.Assertion]:
        return self._store_client.list_confdb_schemas(name=name)

    @override
    def _build_assertion(self, assertion: models.EditableAssertion) -> models.Assertion:
        if not isinstance(assertion, self._editable_assertion_class):
             raise TypeError(
                 f"Expected {self._editable_assertion_class.__name__}, "
                 f"got {type(assertion).__name__}"
             )
        return self._store_client.build_confdb_schema(confdb_schema=assertion)

    @override
    def _post_assertion(self, assertion_data: bytes) -> models.Assertion:
        return self._store_client.post_confdb_schema(confdb_schema_data=assertion_data)

    @override
    def _normalize_assertions(
        self, assertions: list[models.Assertion]
    ) -> tuple[list[str], list[list[Any]]]:
        headers = ["Account ID", "Name", "Revision", "When"]
        confdb_schema = [
            [
                assertion.account_id,
                assertion.name,
                assertion.revision,
                assertion.timestamp[:10],
            ]
            for assertion in assertions
            if isinstance(assertion, models.ConfdbSchemaAssertion)
        ]
        return headers, confdb_schema

    @override
    def _generate_yaml_from_model(self, assertion: models.Assertion) -> str:
        if not isinstance(assertion, models.ConfdbSchemaAssertion):
            raise TypeError(
                f"Expected ConfdbSchemaAssertion, got {type(assertion).__name__}"
            )

        # --- Base Info ---
        # Base fields should always come first
        base_yaml = _TPL_BASE.format(
            account_id=assertion.account_id,
            name=assertion.name
        )
        # Revision comment follows base info
        revision_comment = _TPL_REVISION_COMMENT.format(revision=assertion.revision)

        # Use model_dump to get a dict representation, excluding fields handled elsewhere
        exclude_fields = {
            "account_id", "name", "revision", "headers", "timestamp", "type", "body",
            "authority_id" # <--- Added authority_id here
        }
        model_dict = assertion.model_dump(
            mode="json",
            exclude_unset=True,
            exclude=exclude_fields
        )

        # Convert the remaining relevant parts to OrderedDict recursively
        ordered_model_dict = _dict_to_ordered_dict(model_dict, ordered_keys=TOP_LEVEL_ORDER)


        # --- Dump the ordered dictionary ---
        # Use the custom OrderedDumper
        main_yaml_part = yaml.dump(
            ordered_model_dict,
            Dumper=OrderedDumper,
            default_flow_style=False,
            allow_unicode=True # Ensure unicode characters are handled correctly
        ).rstrip("\n")


        # --- Combine base, revision comment, and main content ---
        yaml_parts = [base_yaml, revision_comment, main_yaml_part]


        # --- Body block (optional, handled manually) ---
        body_data = assertion.body # Get original body string
        if body_data:
            indented_body = textwrap.indent(body_data, "  ") # Body content indent: 2 spaces
            body_block_manual = f"body: |\n{indented_body}"
            yaml_parts.append(body_block_manual) # Append body block last

        # --- Assemble final YAML ---
        # Filter out empty strings that might result from empty dumps
        final_yaml = "\n".join(filter(None, yaml_parts)) + "\n"

        return final_yaml


    @override
    def _generate_yaml_from_template(self, name: str, account_id: str) -> str:
        # Assemble parts for a new schema
        # Base YAML part
        base_yaml = _TPL_BASE.format(account_id=account_id, name=name)
        # Revision comment
        revision_comment = _TPL_REVISION_COMMENT.format(revision=1)
        # Default views and body stripped of trailing newlines
        views_part = _CONFDB_SCHEMA_VIEWS_TEMPLATE_DEFAULT.rstrip("\n")
        body_part = _CONFDB_SCHEMA_BODY_TEMPLATE_DEFAULT.rstrip("\n")

        # Combine in the desired order
        yaml_parts = [base_yaml, revision_comment, views_part, body_part]

        # Join parts with newline, ensure single final newline
        return "\n".join(yaml_parts) + "\n"

    @override
    def _get_success_message(self, assertion: models.Assertion) -> str:
        if not isinstance(assertion, models.ConfdbSchemaAssertion):
            raise TypeError(
                f"Expected ConfdbSchemaAssertion, got {type(assertion).__name__}"
            )
        return f"Successfully created revision {assertion.revision!r} for {assertion.name!r}."

