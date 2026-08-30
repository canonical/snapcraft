# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2026 Canonical Ltd.
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

"""Tests for the validation sets service."""

import textwrap
from contextlib import nullcontext
from unittest import mock

import pytest

from snapcraft import errors
from snapcraft.models import EditableValidationSetAssertion, ValidationSetAssertion


def test_validation_sets_service_type(fake_services):
    validation_sets_service = fake_services.get("validation_sets")

    assert validation_sets_service._assertion_name == "validation-set"


def test_editable_assertion_class(fake_services):
    validation_sets_service = fake_services.get("validation_sets")

    assert (
        validation_sets_service._editable_assertion_class
        == EditableValidationSetAssertion
    )


@pytest.mark.parametrize("sequence", [None, 123, "latest"])
def test_get_assertions(fake_services, sequence):
    validation_sets_service = fake_services.get("validation_sets")

    validation_sets_service._get_assertions("test-validation-set", sequence=sequence)

    validation_sets_service._store_client.list_validation_sets.assert_called_once_with(
        name="test-validation-set", sequence=sequence
    )


def test_build_assertion(fake_services):
    validation_sets_service = fake_services.get("validation_sets")
    mock_assertion = mock.Mock(spec=ValidationSetAssertion)

    validation_sets_service._build_assertion(mock_assertion)

    validation_sets_service._store_client.build_validation_set.assert_called_once_with(
        validation_set=mock_assertion
    )


def test_post_assertions(fake_services):
    validation_sets_service = fake_services.get("validation_sets")
    validation_sets_service._post_assertion(b"test-assertion-data")

    validation_sets_service._store_client.post_validation_set.assert_called_once_with(
        validation_set_data=b"test-assertion-data"
    )


def test_normalize_assertions_empty(fake_services, check):
    validation_sets_service = fake_services.get("validation_sets")
    headers, validation_sets = validation_sets_service._normalize_assertions([])

    check.equal(headers, ["Account ID", "Name", "Sequence", "Revision", "When"])
    check.equal(validation_sets, [])


def test_normalize_assertions(fake_validation_set_assertion, fake_services, check):
    validation_sets_service = fake_services.get("validation_sets")
    validation_sets = [
        fake_validation_set_assertion(),
        fake_validation_set_assertion(
            account_id="test-account-id-2",
            name="test-validation-set-2",
            revision="100",
            sequence="200",
            timestamp="2026-12-31",
        ),
    ]

    headers, normalized_validation_sets = validation_sets_service._normalize_assertions(
        validation_sets
    )

    check.equal(headers, ["Account ID", "Name", "Sequence", "Revision", "When"])
    check.equal(
        normalized_validation_sets,
        [
            ["test-account-id", "test-validation-set", 5, "4", "2026-01-01"],
            ["test-account-id-2", "test-validation-set-2", 200, "100", "2026-12-31"],
        ],
    )


def test_generate_yaml_from_model(fake_validation_set_assertion, fake_services):
    validation_sets_service = fake_services.get("validation_sets")
    yaml_data = validation_sets_service._generate_yaml_from_model(
        fake_validation_set_assertion()
    )

    assert yaml_data == textwrap.dedent(
        """\
        account-id: test-account-id
        name: test-validation-set
        sequence: 5
        # The revision for this validation set
        # revision: 4
        snaps:
        - name: hello-world
          id: test-snap-id
          presence: required
          revision: 6
          components:
            component-with-revision:
              presence: required
              revision: 10
            component-without-revision: invalid
        """
    )


def test_generate_yaml_from_template(fake_services):
    validation_sets_service = fake_services.get("validation_sets")

    yaml_data = validation_sets_service._generate_yaml_from_template(
        name="test-validation-set", account_id="test-account-id", sequence=100
    )

    expected_yaml = textwrap.dedent(
        """\
        account-id: test-account-id
        name: test-validation-set
        sequence: 100
        # The revision for this validation set
        # revision: 0
        snaps:
          - name: hello  # The name of the snap.
        #    id:   <id>    # The ID of the snap. Optional, defaults to the current ID for
                           # the provided name.
        #    presence: [required|optional|invalid]  # Optional, defaults to required.
        #    revision: <n> # The revision of the snap. Optional.
        #    components: # Constraints to apply to the snap's components. Optional.
        #      <component-name>: [required|optional|invalid] # Short form for specifying the component's presence.
        #      <component-name>: # Long form that allows specifying the component's revision.
        #        presence: [required|optional|invalid] # Presence of the component. Required.
        #        revision: <n> # The revision of the component, required if the snap's revision is given.
                               # Otherwise, not allowed.
        """
    )

    assert yaml_data.strip() == expected_yaml.strip()


@pytest.mark.parametrize("revision", [None, 10])
def test_get_success_message(revision, fake_validation_set_assertion, fake_services):
    validation_sets_service = fake_services.get("validation_sets")
    message = validation_sets_service._get_success_message(
        fake_validation_set_assertion(revision=revision)
    )

    assert (
        message
        == f"Successfully created sequence 5 revision {revision or 0} for 'test-validation-set'."
    )


@pytest.mark.parametrize(
    ("old", "new", "expectation"),
    [
        pytest.param(
            10, 9, pytest.raises(errors.SnapcraftAssertionWarning), id="decrement"
        ),
        pytest.param(
            10, 10, pytest.raises(errors.SnapcraftAssertionWarning), id="no-change"
        ),
        pytest.param(10, 11, nullcontext(), id="increment"),
    ],
)
def test_validate_assertion_warning(
    old, new, expectation, fake_services, fake_validation_set_assertion
):
    validation_sets_service = fake_services.get("validation_sets")
    validation_set = fake_validation_set_assertion(sequence=new)

    with expectation:
        validation_sets_service._validate_assertion(validation_set, sequence=old)
