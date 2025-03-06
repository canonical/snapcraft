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

from snapcraft.models import Confdb, ConfdbAssertion, EditableConfdbAssertion
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


def test_confdb_defaults(check):
    """Test default values of the Confdb model."""
    confdb = Confdb.unmarshal({"storage": "test-storage"})

    check.is_none(confdb.request)
    check.is_none(confdb.access)
    check.is_none(confdb.content)


def test_confdb_nested(check):
    """Test that nested confdbs are supported."""
    confdb = Confdb.unmarshal(
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

    check.equal(confdb.request, "test-request")
    check.equal(confdb.storage, "test-storage")
    check.equal(confdb.access, "read")
    check.equal(
        confdb.content,
        [Confdb(request="nested-request", storage="nested-storage", access="write")],
    )


def test_editable_confdb_assertion_defaults(check):
    """Test default values of the EditableConfdbAssertion model."""
    assertion = EditableConfdbAssertion.unmarshal(
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


def test_editable_confdb_assertion_marshal_as_str():
    """Cast all scalars to string when marshalling."""
    assertion = EditableConfdbAssertion.unmarshal(
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


def test_confdb_assertion_defaults(check):
    """Test default values of the ConfdbAssertion model."""
    assertion = ConfdbAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-confdb",
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "confdb",
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

    check.is_none(assertion.body)
    check.is_none(assertion.body_length)
    check.is_none(assertion.sign_key_sha3_384)
    check.equal(assertion.revision, 0)


def test_confdb_assertion_marshal_as_str():
    """Cast all scalars to strings when marshalling."""
    assertion = ConfdbAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-confdb",
            "revision": 10,
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "confdb",
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
