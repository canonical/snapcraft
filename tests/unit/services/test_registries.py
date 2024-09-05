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

"""Tests for the registries service."""


def test_registries_service_type(registries_service):
    assert registries_service._assertion_type == "registries"


def test_get_assertions(registries_service):
    registries_service._get_assertions("test-registry")

    registries_service._store_client.list_registries.assert_called_once_with(
        name="test-registry"
    )


def test_normalize_assertions_empty(registries_service, check):
    headers, registries = registries_service._normalize_assertions([])

    check.equal(headers, ["Account ID", "Name", "Revision", "When"])
    check.equal(registries, [])


def test_normalize_assertions(fake_registry_assertion_data, registries_service, check):
    registries = [
        fake_registry_assertion_data,
        fake_registry_assertion_data.copy(
            update={
                "account_id": "test-account-id-2",
                "name": "test-registry-2",
                "revision": 100,
                "timestamp": "2024-12-31",
            }
        ),
    ]

    headers, normalized_registries = registries_service._normalize_assertions(
        registries
    )

    check.equal(headers, ["Account ID", "Name", "Revision", "When"])
    check.equal(
        normalized_registries,
        [
            ["test-account-id", "test-registry", 0, "2024-01-01"],
            ["test-account-id-2", "test-registry-2", 100, "2024-12-31"],
        ],
    )
