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

import textwrap
from typing import Any

from craft_application.util import dump_yaml
from typing_extensions import override

from snapcraft import models
from snapcraft.services import Assertion

_CONFDB_SCHEMA_TEMPLATE = textwrap.dedent(
    """\
    account-id: {account_id}
    name: {name}{summary}
    # The revision for this confdb-schema
    # revision: {revision}
    {views}
    {body}
    """
)


_CONFDB_SCHEMA_VIEWS_TEMPLATE = textwrap.dedent(
    """\
    views:
      wifi-setup:
        summary: Summary of the view.
        rules:
          - request: ssids
            storage: wifi.ssids
            access: read
    """
)


_CONFDB_SCHEMA_BODY_TEMPLATE = textwrap.dedent(
    """\
    body: |-
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


class ConfdbSchemas(
    Assertion[models.EditableConfdbSchemaAssertion, models.ConfdbSchemaAssertion]
):
    """Service for interacting with confdb schemas."""

    @property
    @override
    def _assertion_name(self) -> str:
        return "confdb-schema"

    @property
    @override
    def _editable_assertion_class(self) -> type[models.EditableConfdbSchemaAssertion]:
        return models.EditableConfdbSchemaAssertion

    @override
    def _get_assertions(
        self, name: str | None = None, **kwargs: dict[str, Any]
    ) -> list[models.ConfdbSchemaAssertion]:
        return self._store_client.list_confdb_schemas(name=name)

    @override
    def _build_assertion(
        self, assertion: models.EditableConfdbSchemaAssertion
    ) -> models.ConfdbSchemaAssertion:
        return self._store_client.build_confdb_schema(confdb_schema=assertion)

    @override
    def _post_assertion(self, assertion_data: bytes) -> models.ConfdbSchemaAssertion:
        return self._store_client.post_confdb_schema(confdb_schema_data=assertion_data)

    @override
    def _normalize_assertions(
        self, assertions: list[models.ConfdbSchemaAssertion]
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
        ]

        return headers, confdb_schema

    @override
    def _generate_yaml_from_model(self, assertion: models.ConfdbSchemaAssertion) -> str:
        # Include the summary field only when it's explicitly set (to avoid outputting 'None' in the generated YAML)
        summary = f"\nsummary: {assertion.summary}" if assertion.summary else ""
        return _CONFDB_SCHEMA_TEMPLATE.format(
            account_id=assertion.account_id,
            views=dump_yaml(
                {"views": assertion.marshal().get("views")}, default_flow_style=False
            ),
            body=dump_yaml({"body": assertion.body}, default_flow_style=False),
            name=assertion.name,
            revision=assertion.revision,
            summary=summary,
        )

    @override
    def _generate_yaml_from_template(
        self, name: str, account_id: str, **kwargs: dict[str, Any]
    ) -> str:
        return _CONFDB_SCHEMA_TEMPLATE.format(
            account_id=account_id,
            views=_CONFDB_SCHEMA_VIEWS_TEMPLATE,
            body=_CONFDB_SCHEMA_BODY_TEMPLATE,
            name=name,
            revision=1,
            summary="\nsummary: Summary of the confdb-schema",
        )

    @override
    def _get_success_message(self, assertion: models.ConfdbSchemaAssertion) -> str:
        return f"Successfully created revision {assertion.revision!r} for {assertion.name!r}."
