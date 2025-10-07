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

    check.is_none(confdb_schema.request)
    check.is_none(confdb_schema.access)
    check.is_none(confdb_schema.content)


def test_confdb_schema_nested(check):
    """Test that nested confdb schemas are supported."""
    confdb_schema = ConfdbSchema.unmarshal(
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
    )

    check.equal(confdb_schema.request, "test-request")
    check.equal(confdb_schema.storage, "test-storage")
    check.equal(confdb_schema.access, "read")
    check.equal(
        confdb_schema.content,
        [
            ConfdbSchema(
                request="nested-request", storage="nested-storage", access="write"
            )
        ],
    )


def test_editable_confdb_schema_assertion_defaults(check):
    """Test default values of the EditableConfdbSchemaAssertion model."""
    assertion = EditableConfdbSchemaAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "name": "test-confdb",
            "views": {
                "wifi-setup": {
                    "rules": [
                        {
                            "storage": "wifi.ssids",
                        }
                    ]
                }
            },
        }
    )

    check.equal(assertion.revision, 0)
    check.is_none(assertion.body)


def test_editable_confdb_schema_assertion_marshal_as_str():
    """Cast all scalars to string when marshalling."""
    assertion = EditableConfdbSchemaAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "name": "test-confdb",
            "revision": 10,
            "views": {
                "wifi-setup": {
                    "rules": [
                        {
                            "storage": "wifi.ssids",
                        }
                    ]
                }
            },
        }
    )

    assertion_dict = assertion.marshal_scalars_as_strings()

    assert assertion_dict["revision"] == "10"


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
                    ],
                }
            },
        }
    )

    check.is_none(assertion.body)
    check.is_none(assertion.body_length)
    check.is_none(assertion.sign_key_sha3_384)
    check.equal(assertion.revision, 0)
    check.is_none(assertion.summary)
    check.is_none(assertion.views["wifi-setup"].summary)


def test_confdb_schema_assertion_marshal_as_str():
    """Cast all scalars to strings when marshalling."""
    assertion = ConfdbSchemaAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-confdb",
            "revision": 10,
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "confdb-schema",
            "views": {
                "wifi-setup": {
                    "rules": [
                        {
                            "storage": "wifi.ssids",
                        }
                    ]
                }
            },
        }
    )

    assertion_dict = assertion.marshal_scalars_as_strings()

    assert assertion_dict["revision"] == "10"


def test_confdb_schema_assertion_with_summary(check):
    """Test that summaries are set correctly when provided."""
    assertion = ConfdbSchemaAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-confdb",
            "summary": "This is a test confdb-schema summary.",
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "confdb-schema",
            "views": {
                "wifi-setup": {
                    "summary": "This is a test views summary.",
                    "rules": [
                        {
                            "access": "read-write",
                            "request": "ssids",
                            "storage": "wifi.ssids",
                        }
                    ],
                }
            },
        }
    )

    check.equal(assertion.summary, "This is a test confdb-schema summary.")
    check.equal(assertion.views["wifi-setup"].summary, "This is a test views summary.")
