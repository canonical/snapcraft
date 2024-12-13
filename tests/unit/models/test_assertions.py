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

from snapcraft.models import EditableRegistryAssertion, Registry, RegistryAssertion
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


def test_registry_defaults(check):
    """Test default values of the Registry model."""
    registry = Registry.unmarshal({"storage": "test-storage"})

    check.is_none(registry.request)
    check.is_none(registry.access)
    check.is_none(registry.content)


def test_registry_nested(check):
    """Test that nested registries are supported."""
    registry = Registry.unmarshal(
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

    check.equal(registry.request, "test-request")
    check.equal(registry.storage, "test-storage")
    check.equal(registry.access, "read")
    check.equal(
        registry.content,
        [Registry(request="nested-request", storage="nested-storage", access="write")],
    )


def test_editable_registry_assertion_defaults(check):
    """Test default values of the EditableRegistryAssertion model."""
    assertion = EditableRegistryAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "name": "test-registry",
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


def test_editable_registry_assertion_marshal_as_str():
    """Cast all scalars to string when marshalling."""
    assertion = EditableRegistryAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "name": "test-registry",
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


def test_registry_assertion_defaults(check):
    """Test default values of the RegistryAssertion model."""
    assertion = RegistryAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-registry",
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "registry",
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


def test_registry_assertion_marshal_as_str():
    """Cast all scalars to strings when marshalling."""
    assertion = RegistryAssertion.unmarshal(
        {
            "account_id": "test-account-id",
            "authority_id": "test-authority-id",
            "name": "test-registry",
            "revision": 10,
            "timestamp": "2024-01-01T10:20:30Z",
            "type": "registry",
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
