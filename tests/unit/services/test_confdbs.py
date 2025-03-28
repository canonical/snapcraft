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

"""Tests for the confdbs service."""

import textwrap
from unittest import mock

from snapcraft.models import ConfdbAssertion, EditableConfdbAssertion


def test_confdbs_service_type(fake_services):
    confdbs_service = fake_services.get("confdbs")

    assert confdbs_service._assertion_name == "confdbs set"


def test_editable_assertion_class(fake_services):
    confdbs_service = fake_services.get("confdbs")

    assert confdbs_service._editable_assertion_class == EditableConfdbAssertion


def test_get_assertions(fake_services):
    confdbs_service = fake_services.get("confdbs")

    confdbs_service._get_assertions("test-confdb")

    confdbs_service._store_client.list_confdbs.assert_called_once_with(
        name="test-confdb"
    )


def test_build_assertion(fake_services):
    confdbs_service = fake_services.get("confdbs")
    mock_assertion = mock.Mock(spec=ConfdbAssertion)

    confdbs_service._build_assertion(mock_assertion)

    confdbs_service._store_client.build_confdbs.assert_called_once_with(
        confdbs=mock_assertion
    )


def test_post_assertions(fake_services):
    confdbs_service = fake_services.get("confdbs")
    confdbs_service._post_assertion(b"test-assertion-data")

    confdbs_service._store_client.post_confdbs.assert_called_once_with(
        confdbs_data=b"test-assertion-data"
    )


def test_normalize_assertions_empty(fake_services, check):
    confdbs_service = fake_services.get("confdbs")
    headers, confdbs = confdbs_service._normalize_assertions([])

    check.equal(headers, ["Account ID", "Name", "Revision", "When"])
    check.equal(confdbs, [])


def test_normalize_assertions(fake_confdb_assertion, fake_services, check):
    confdbs_service = fake_services.get("confdbs")
    confdbs = [
        fake_confdb_assertion(),
        fake_confdb_assertion(
            account_id="test-account-id-2",
            name="test-confdb-2",
            revision=100,
            timestamp="2024-12-31",
        ),
    ]

    headers, normalized_confdbs = confdbs_service._normalize_assertions(confdbs)

    check.equal(headers, ["Account ID", "Name", "Revision", "When"])
    check.equal(
        normalized_confdbs,
        [
            ["test-account-id", "test-confdb", 0, "2024-01-01"],
            ["test-account-id-2", "test-confdb-2", 100, "2024-12-31"],
        ],
    )


def test_generate_yaml_from_model(fake_confdb_assertion, fake_services):
    confdbs_service = fake_services.get("confdbs")
    assertion = fake_confdb_assertion(
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
    yaml_data = confdbs_service._generate_yaml_from_model(assertion)

    assert yaml_data == textwrap.dedent(
        """\
        account-id: test-account-id
        name: test-confdb
        # The revision for this confdbs set
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


def test_get_success_message(fake_confdb_assertion, fake_services):
    confdbs_service = fake_services.get("confdbs")
    message = confdbs_service._get_success_message(fake_confdb_assertion(revision=10))

    assert message == "Successfully created revision 10 for 'test-confdb'."
