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

from snapcraft.models import EditableRegistryAssertion, Registry, RegistryAssertion


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

    check.is_none(assertion.summary)
    check.equal(assertion.revision, 0)
    check.is_none(assertion.body)


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
    check.is_none(assertion.summary)
    check.equal(assertion.revision, 0)
