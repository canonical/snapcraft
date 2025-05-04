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

import pytest

from snapcraft.models import EditableConfdbSchemaAssertion


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
    mock_assertion = mock.Mock(spec=EditableConfdbSchemaAssertion) # Use Editable for build

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

# Use parametrization to test with and without summary data
# Adjusted expected YAML to match Pydantic model order (request before summary)
# and standard YAML quoting (quotes only when needed).
@pytest.mark.parametrize(
    "summary_data, expected_yaml",
    [
        pytest.param(
            {}, # No summaries provided
            textwrap.dedent("""\
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
                """),
            id="no_summaries",
        ),
        pytest.param(
            {
                "top_level_summary": "Top Level Summary.",
                "view_summary": "View Summary.",
                "rule_summary": "Rule Summary.",
                "nested_rule_summary": "Nested Rule Summary.",
            },
             # Expected YAML with summaries, request before summary, standard quotes
             textwrap.dedent("""\
                account-id: test-account-id
                name: test-confdb
                # The revision for this confdb-schema
                # revision: 10
                summary: Top Level Summary.
                views:
                  wifi-setup:
                    summary: View Summary.
                    rules:
                    - request: test-request
                      storage: test-storage
                      access: read
                      summary: Rule Summary.
                      content:
                      - request: nested-request
                        storage: nested-storage
                        access: write
                        summary: Nested Rule Summary.
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
                """),
            id="with_summaries",
        ),
    ],
)
def test_generate_yaml_from_model(
    summary_data,
    expected_yaml,
    fake_confdb_schema_assertion,
    fake_services,
    check,
):
    """Test YAML generation with and without summary fields."""
    confdb_schemas_service = fake_services.get("confdb_schemas")

    # --- Prepare Assertion Data ---
    views_data = {
        "wifi-setup": {
            "rules": [
                {
                    "request": "test-request", # Ensure order matches model if needed
                    "storage": "test-storage",
                    "access": "read",
                    "content": [
                        {
                            "request": "nested-request", # Ensure order matches model
                            "storage": "nested-storage",
                            "access": "write",
                        }
                    ],
                }
            ]
        }
    }
    # Inject summaries if present in summary_data - order might be determined by dict insertion
    if "view_summary" in summary_data:
        # Insert summary *after* other view keys if that's the expected dump order
        views_data["wifi-setup"]["summary"] = summary_data["view_summary"]
    if "rule_summary" in summary_data:
        views_data["wifi-setup"]["rules"][0]["summary"] = summary_data["rule_summary"]
    if "nested_rule_summary" in summary_data:
        views_data["wifi-setup"]["rules"][0]["content"][0]["summary"] = summary_data[
            "nested_rule_summary"
        ]

    assertion_kwargs = {
        "revision": "10",
        "views": views_data,
        "body": """{
  "storage": {
    "schema": {
      "wifi": {
        "values": "any"
      }
    }
  }
}""", # Body with double quotes
    }
    if "top_level_summary" in summary_data:
        assertion_kwargs["summary"] = summary_data["top_level_summary"]

    assertion = fake_confdb_schema_assertion(**assertion_kwargs)

    # --- Generate YAML ---
    yaml_data = confdb_schemas_service._generate_yaml_from_model(assertion)

    # --- Assertions ---
    # Ensure expected YAML has a trailing newline like the generated one
    expected_yaml_with_newline = expected_yaml.rstrip() + "\n"


    # Useful for debugging YAML differences
    # print("------ GOT ------")
    # print(repr(yaml_data))
    # print("------ EXPECTED ------")
    # print(repr(expected_yaml_with_newline))

    check.equal(yaml_data, expected_yaml_with_newline)


def test_get_success_message(fake_confdb_schema_assertion, fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    message = confdb_schemas_service._get_success_message(
        fake_confdb_schema_assertion(revision=10)
    )

    assert message == "Successfully created revision 10 for 'test-confdb'."

# Test the template generation (should not include summary initially)
def test_generate_yaml_from_template(fake_services):
    confdb_schemas_service = fake_services.get("confdb_schemas")
    yaml_data = confdb_schemas_service._generate_yaml_from_template(
        name="new-schema", account_id="new-account"
    )

    expected_yaml = textwrap.dedent("""\
        account-id: new-account
        name: new-schema
        # The revision for this confdb-schema
        # revision: 1
        views:
          wifi-setup:
            rules:
              - request: ssids
                storage: wifi.ssids
                access: read
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
        """) # Note the absence of the top-level 'summary:' key
    assert yaml_data == expected_yaml.rstrip() + "\n"

