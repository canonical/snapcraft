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

"""Tests for the confdb schemas service."""

import textwrap
from unittest import mock

from snapcraft.models import ConfdbSchemaAssertion, EditableConfdbSchemaAssertion


def test_confdb_schemas_service_type(fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")

    assert confdb_schemas_service._assertion_name == "confdb-schema"


def test_editable_assertion_class(fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")

    assert (
        confdb_schemas_service._editable_assertion_class
        == EditableConfdbSchemaAssertion
    )


def test_get_assertions(fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")

    confdb_schemas_service._get_assertions("test-confdb")

    confdb_schemas_service._store_client.list_confdb_schemas.assert_called_once_with(
        name="test-confdb"
    )


def test_build_assertion(fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    mock_assertion = mock.Mock(spec=ConfdbSchemaAssertion)

    confdb_schemas_service._build_assertion(mock_assertion)

    confdb_schemas_service._store_client.build_confdb_schema.assert_called_once_with(
        confdb_schema=mock_assertion
    )


def test_post_assertions(fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    confdb_schemas_service._post_assertion(b"test-assertion-data")

    confdb_schemas_service._store_client.post_confdb_schema.assert_called_once_with(
        confdb_schema_data=b"test-assertion-data"
    )


def test_normalize_assertions_empty(fake_services, check):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    headers, confdb_schemas = confdb_schemas_service._normalize_assertions([])

    check.equal(headers, ["Account ID", "Name", "Revision", "When"])
    check.equal(confdb_schemas, [])


def test_normalize_assertions(fake_confdb_schema_assertion, fake_services, check):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    confdb_schemas = [
        fake_confdb_schema_assertion(),
        fake_confdb_schema_assertion(
            account_id="test-account-id-2",
            name="test-confdb-2",
            revision=100,
            timestamp="2024-12-31",
        ),
    ]

    headers, normalized_confdb_schemas = confdb_schemas_service._normalize_assertions(
        confdb_schemas
    )

    check.equal(headers, ["Account ID", "Name", "Revision", "When"])
    check.equal(
        normalized_confdb_schemas,
        [
            ["test-account-id", "test-confdb", 0, "2024-01-01"],
            ["test-account-id-2", "test-confdb-2", 100, "2024-12-31"],
        ],
    )


def test_generate_yaml_from_model(fake_confdb_schema_assertion, fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    assertion = fake_confdb_schema_assertion(
        revision="10",
        views={
            "wifi-setup": {
                "rules": [
                    {
                        "request": "test-request",
                        "storage": "test-storage",
                        "access": "read",
                        "content": [
                            {
                                "request": "nested-request",
                                "storage": "nested-storage",
                                "access": "write",
                            }
                        ],
                    }
                ]
            }
        },
        body=(
            "{\n  'storage': {\n    'schema': {\n      'wifi': {\n        "
            "'values': 'any'\n      }\n    }\n  }\n}"
        ),
    )
    yaml_data = confdb_schemas_service._generate_yaml_from_model(assertion)

    assert yaml_data == textwrap.dedent(
        """\
        account-id: test-account-id
        name: test-confdb
        # The revision for this confdb-schema
        # revision: 10
        views:
          wifi-setup:
            rules:
            - request: test-request
              storage: test-storage
              access: read
              content:
              - request: nested-request
                storage: nested-storage
                access: write

        body: |-
          {
            'storage': {
              'schema': {
                'wifi': {
                  'values': 'any'
                }
              }
            }
          }

          """
    )


def test_generate_yaml_from_model_with_summary(
    fake_confdb_schema_assertion, fake_services
):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    assertion = fake_confdb_schema_assertion(
        revision="10",
        summary="This is a test confdb-schema summary.",
        views={
            "wifi-setup": {
                "summary": "This is a test views summary.",
                "rules": [
                    {
                        "request": "test-request",
                        "storage": "test-storage",
                        "access": "read",
                        "content": [
                            {
                                "request": "nested-request",
                                "storage": "nested-storage",
                                "access": "write",
                            }
                        ],
                    }
                ],
            }
        },
        body=(
            "{\n  'storage': {\n    'schema': {\n      'wifi': {\n        "
            "'values': 'any'\n      }\n    }\n  }\n}"
        ),
    )
    yaml_data = confdb_schemas_service._generate_yaml_from_model(assertion)

    assert yaml_data == textwrap.dedent(
        """\
        account-id: test-account-id
        name: test-confdb
        summary: This is a test confdb-schema summary.
        # The revision for this confdb-schema
        # revision: 10
        views:
          wifi-setup:
            summary: This is a test views summary.
            rules:
            - request: test-request
              storage: test-storage
              access: read
              content:
              - request: nested-request
                storage: nested-storage
                access: write

        body: |-
          {
            'storage': {
              'schema': {
                'wifi': {
                  'values': 'any'
                }
              }
            }
          }

          """
    )


def test_generate_yaml_from_template(fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")

    yaml_data = confdb_schemas_service._generate_yaml_from_template(
        name="test-confdb", account_id="test-account-id"
    )

    expected_yaml = textwrap.dedent(
        """\
        account-id: test-account-id
        name: test-confdb
        summary: Summary of the confdb-schema
        # The revision for this confdb-schema
        # revision: 1
        views:
          wifi-setup:
            summary: Summary of the view.
            rules:
              - request: ssids
                storage: wifi.ssids
                access: read

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

    assert yaml_data.strip() == expected_yaml.strip()


def test_get_success_message(fake_confdb_schema_assertion, fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    message = confdb_schemas_service._get_success_message(
        fake_confdb_schema_assertion(revision=10)
    )

    assert message == "Successfully created revision 10 for 'test-confdb'."
