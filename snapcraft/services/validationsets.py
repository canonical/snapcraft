# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Service class for validation sets."""

from __future__ import annotations

import textwrap
from typing import Any, cast

from craft_application.util import dump_yaml
from craft_cli import emit
from typing_extensions import override

from snapcraft import errors, models
from snapcraft.services import Assertion

_VALIDATIONS_SET_SNAPS_TEMPLATE = textwrap.dedent(
    """\
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

_VALIDATION_SET_TEMPLATE = textwrap.dedent(
    """\
    account-id: {account_id}
    name: {name}
    sequence: {sequence}
    # The revision for this validation set
    # revision: {revision}
    {snaps}"""
)


class ValidationSets(
    Assertion[models.EditableValidationSetAssertion, models.ValidationSetAssertion]
):
    """Service for interacting with validation sets."""

    @property
    @override
    def _assertion_name(self) -> str:
        return "validation-set"

    @property
    @override
    def _editable_assertion_class(self) -> type[models.EditableValidationSetAssertion]:
        return models.EditableValidationSetAssertion

    @override
    def _get_assertions(
        self, name: str | None = None, **kwargs: dict[str, Any]
    ) -> list[models.ValidationSetAssertion]:
        sequence = cast(str, kwargs.get("sequence")) if kwargs.get("sequence") else None
        return self._store_client.list_validation_sets(name=name, sequence=sequence)

    @override
    def _build_assertion(
        self, assertion: models.EditableValidationSetAssertion
    ) -> models.ValidationSetAssertion:
        return self._store_client.build_validation_set(validation_set=assertion)

    @override
    def _post_assertion(self, assertion_data: bytes) -> models.ValidationSetAssertion:
        return self._store_client.post_validation_set(
            validation_set_data=assertion_data
        )

    @override
    def _normalize_assertions(
        self, assertions: list[models.ValidationSetAssertion]
    ) -> tuple[list[str], list[list[Any]]]:
        headers = ["Account ID", "Name", "Sequence", "Revision", "When"]
        validation_set = [
            [
                assertion.account_id,
                assertion.name,
                assertion.sequence,
                assertion.revision,
                assertion.timestamp[:10],
            ]
            for assertion in assertions
        ]

        return headers, validation_set

    @override
    def _generate_yaml_from_model(
        self, assertion: models.ValidationSetAssertion
    ) -> str:
        revision = assertion.revision
        try:
            snaps = dump_yaml(
                {"snaps": [s.marshal() for s in assertion.snaps]},
                default_flow_style=False,
                allow_unicode=True,
            )
        except IndexError:
            # If there is no assertion for a given sequence, the store API
            # will return an empty list.
            revision = "0"
            snaps = _VALIDATIONS_SET_SNAPS_TEMPLATE

        unverified_validation_sets = _VALIDATION_SET_TEMPLATE.format(
            account_id=assertion.account_id,
            name=assertion.name,
            sequence=assertion.sequence,
            revision=revision,
            snaps=snaps,
        )

        return unverified_validation_sets

    @override
    def _generate_yaml_from_template(self, name: str, account_id: str, **kwargs) -> str:
        return _VALIDATION_SET_TEMPLATE.format(
            account_id=account_id,
            name=name,
            sequence=kwargs.get("sequence"),
            revision="0",
            snaps=_VALIDATIONS_SET_SNAPS_TEMPLATE,
        )

    @override
    def _get_success_message(self, assertion: models.ValidationSetAssertion) -> str:
        return (
            f"Successfully created sequence {assertion.sequence} "
            f"revision {assertion.revision or 0} for {assertion.name!r}."
        )

    @override
    def _validate_assertion(
        self, assertion: models.ValidationSetAssertion, **kwargs: dict[str, Any]
    ) -> None:
        """Validate that the sequence has been incremented.

        :param assertion: The assertion to validate.
        :param kwargs: Additional keyword arguments to use for validation.

        :raises SnapcraftAssertionWarning: If the sequence wasn't incremented.
        """
        new_sequence = assertion.sequence
        old_sequence = cast(int, kwargs.get("sequence", 0))
        emit.debug(f"Sequence updated from {old_sequence} to {new_sequence}")

        if new_sequence <= old_sequence:
            raise errors.SnapcraftAssertionWarning(
                "Warning: The sequence number was not incremented. This prevents automatic reversions to a valid state in "
                "the case of invalid changes or snap refresh failures."
            )
