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


"""Tests for Assertion models."""

import pytest

from snapcraft.models import (
    ConfdbSchema,
    ConfdbSchemaAssertion,
    EditableConfdbSchemaAssertion,
)
from snapcraft.models.assertions import cast_dict_scalars_to_strings


@pytest.mark.parametrize(
    ("input_dict", "expected_dict"),
    [
        pytest.param({}, {}, id="empty"),
        pytest.param(
            {False: False, True: True},
            {"False": "False", "True": "True"},
            id="boolean values",
        ),
        pytest.param(
            {0: 0, None: None, "dict": {}, "list": [], "str": ""},
            ({"0": "0", None: None, "dict": {}, "list": [], "str": ""}),
            id="none-like values",
        ),
        pytest.param(
            {10: 10, 20.0: 20.0, "30": "30", True: True},
            {"10": "10", "20.0": "20.0", "30": "30", "True": "True"},
            id="scalar values",
        ),
        pytest.param(
            {"foo": {"bar": [1, 2.0], "baz": {"qux": True}}},
            {"foo": {"bar": ["1", "2.0"], "baz": {"qux": "True"}}},
            id="nested data structures",
        ),
    ],
)
def test_cast_dict_scalars_to_strings(input_dict, expected_dict):
    actual = cast_dict_scalars_to_strings(input_dict)

    assert actual == expected_dict


def test_confdb_schema_defaults(check):
    """Test default values of the ConfdbSchema model."""
    confdb_schema = ConfdbSchema.unmarshal({"storage": "test-storage"})

    check.is_none(confdb_schema.summary)
    check.is_none(confdb_schema.request)
    check.is_none(confdb_schema.access)
    check.is_none(confdb_schema.content)


def test_confdb_schema_nested(check):
    """Test that nested confdb schemas are supported, including summary."""
    confdb_schema = ConfdbSchema.unmarshal(
        {
            "summary": "Top rule summary",
            "request": "test-request",
            "storage": "test-storage",
            "access": "read",
            "content": [
                {
                    "summary": "Nested rule summary",
                    "request": "nested-request",
                    "storage": "nested-storage",
                    "access": "write",
                    "content": [{"storage": "deep-nested-storage"}] # Test summary defaults to None here
                }
            ],
        }
    )

    check.equal(confdb_schema.summary, "Top rule summary")
    check.equal(confdb_schema.request, "test-request")
    check.equal(confdb_schema.storage, "test-storage")
    check.equal(confdb_schema.access, "read")
    check.is_not_none(confdb_schema.content)
    check.equal(len(confdb_schema.content), 1)

    nested_rule = confdb_schema.content[0]
    check.equal(nested_rule.summary, "Nested rule summary")
    check.equal(nested_rule.request, "nested-request")
    check.equal(nested_rule.storage, "nested-storage")
    check.equal(nested_rule.access, "write")
    check.is_not_none(nested_rule.content)
    check.equal(len(nested_rule.content), 1)

    deep_nested_rule = nested_rule.content[0]
    check.is_none(deep_nested_rule.summary) # Check default None
    check.equal(deep_nested_rule.storage, "deep-nested-storage")
    check.is_none(deep_nested_rule.request)
    check.is_none(deep_nested_rule.access)
    check.is_none(deep_nested_rule.content)


def test_editable_confdb_schema_assertion_defaults(check):
    """Test default values of the EditableConfdbSchemaAssertion model."""
    assertion_data = {
        "account_id": "test-account-id",
        "name": "test-confdb",
        "views": {
            "wifi-setup": {
                "rules": [
                    {
                        "storage": "wifi.ssids",
                    }
                ]
            },
            "another-view": { # Check summary defaults to None here
                 "rules": [
                    {
                        "storage": "other.config",
                    }
                ]
            }
        },
    }
    assertion = EditableConfdbSchemaAssertion.unmarshal(assertion_data)

    check.is_none(assertion.summary)
    check.equal(assertion.revision, 0)
    check.is_none(assertion.body)
    check.is_not_none(assertion.views)

    wifi_view = assertion.views.get("wifi-setup")
    check.is_not_none(wifi_view)
    check.is_none(wifi_view.summary) # Check default None for view summary
    check.is_not_none(wifi_view.rules)
    check.equal(len(wifi_view.rules), 1)
    check.is_none(wifi_view.rules[0].summary) # Check default None for rule summary

    another_view = assertion.views.get("another-view")
    check.is_not_none(another_view)
    check.is_none(another_view.summary) # Check default None
    check.is_not_none(another_view.rules)


def test_editable_confdb_schema_assertion_marshal_as_str():
    """Cast all scalars to string when marshalling, including summary."""
    assertion = EditableConfdbSchemaAssertion.unmarshal(
        {
            "summary": "Top level summary",
            "account_id": "test-account-id",
            "name": "test-confdb",
            "revision": 10,
            "views": {
                "wifi-setup": {
                    "summary": "View summary",
                    "rules": [
                        {
                            "summary": "Rule summary",
                            "storage": "wifi.ssids",
                        }
                    ]
                },
                "no-summary-view": {
                    "rules": [{"storage": "other.things"}] # Check None summary
                }
            },
             "body": "{}" # Add a body to check it's handled
        }
    )

    assertion_dict = assertion.marshal_scalars_as_strings()

    assert assertion_dict["summary"] == "Top level summary"
    assert assertion_dict["revision"] == "10"
    assert assertion_dict["body"] == "{}"
    assert assertion_dict["views"]["wifi-setup"]["summary"] == "View summary"
    assert "summary" not in assertion_dict["views"]["no-summary-view"]
    assert assertion_dict["views"]["wifi-setup"]["rules"][0]["summary"] == "Rule summary"
    assert "summary" not in assertion_dict["views"]["no-summary-view"]["rules"][0]


def test_confdb_schema_assertion_defaults(check):
    """Test default values of the ConfdbSchemaAssertion model."""
    assertion = ConfdbSchemaAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-confdb",
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "confdb-schema",
            "views": {
                "wifi-setup": {
                    "rules": [
                        {
                            "access": "read-write",
                            "request": "ssids",
                            "storage": "wifi.ssids",
                        }
                    ]
                }
            },
        }
    )

    check.is_none(assertion.summary) # Check top-level summary default
    check.is_none(assertion.body)
    check.is_none(assertion.body_length)
    check.is_none(assertion.sign_key_sha3_384)
    check.equal(assertion.revision, 0)

    wifi_view = assertion.views.get("wifi-setup")
    check.is_not_none(wifi_view)
    check.is_none(wifi_view.summary) # Check view summary default
    check.is_not_none(wifi_view.rules)
    check.equal(len(wifi_view.rules), 1)
    check.is_none(wifi_view.rules[0].summary) # Check rule summary default


def test_confdb_schema_assertion_marshal_as_str():
    """Cast all scalars to strings when marshalling, including summary."""
    assertion = ConfdbSchemaAssertion.unmarshal(
        {
            "summary": "Full assertion summary",
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-confdb",
            "revision": 10,
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "confdb-schema",
            "views": {
                "wifi-setup": {
                    "summary": "View summary here",
                    "rules": [
                        {
                            "summary": "Rule summary inside",
                            "storage": "wifi.ssids",
                        }
                    ]
                }
            },
            "body": "{}",
            "body_length": 2,
            "sign_key_sha3_384": "some-key-id"
        }
    )

    assertion_dict = assertion.marshal_scalars_as_strings()

    assert assertion_dict["summary"] == "Full assertion summary"
    assert assertion_dict["revision"] == "10"
    assert assertion_dict["body"] == "{}"
    assert assertion_dict["body_length"] == "2" # Check numeric cast
    assert assertion_dict["views"]["wifi-setup"]["summary"] == "View summary here"
    assert assertion_dict["views"]["wifi-setup"]["rules"][0]["summary"] == "Rule summary inside"
    assert assertion_dict["sign_key_sha3_384"] == "some-key-id"
    assert assertion_dict["timestamp"] == "2024-01-01T10:20:30Z"
    assert assertion_dict["type"] == "confdb-schema"

def test_confdb_schema_assertion_with_mixed_summaries(check):
    """Test assertion unmarshalling with summaries present at different levels."""
    assertion_data = {
        "summary": "Top-level only",
        "account_id": "test-account-id",
        "authority_id": "test-authority-id",
        "name": "test-confdb-top",
        "timestamp": "2024-01-01T10:20:31Z",
        "type": "confdb-schema",
        "views": {
            "view1": {"rules": [{"storage": "storage1"}]},
            "view2": {
                "summary": "View-level only",
                "rules": [{"storage": "storage2"}]
            },
            "view3": {
                "rules": [{"summary": "Rule-level only", "storage": "storage3"}]
            },
            "view4": {
                "summary": "View and Rule",
                "rules": [{"summary": "Rule in View4", "storage": "storage4"}]
            }
        }
    }
    assertion = ConfdbSchemaAssertion.unmarshal(assertion_data)

    check.equal(assertion.summary, "Top-level only")
    check.is_none(assertion.views["view1"].summary)
    check.is_none(assertion.views["view1"].rules[0].summary)
    check.equal(assertion.views["view2"].summary, "View-level only")
    check.is_none(assertion.views["view2"].rules[0].summary)
    check.is_none(assertion.views["view3"].summary)
    check.equal(assertion.views["view3"].rules[0].summary, "Rule-level only")
    check.equal(assertion.views["view4"].summary, "View and Rule")
    check.equal(assertion.views["view4"].rules[0].summary, "Rule in View4")
