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

"""Tests for the abstract assertions service."""

import textwrap
from typing import Any

import pytest
from craft_application.models import CraftBaseModel
from typing_extensions import override

from snapcraft import const, errors


class FakeAssertion(CraftBaseModel):
    """Fake assertion model."""

    test_field_1: str
    test_field_2: int


@pytest.fixture
def fake_assertion_service(default_factory):
    from snapcraft.application import APP_METADATA
    from snapcraft.services import Assertion

    class FakeAssertionService(Assertion):
        @property
        @override
        def _assertion_name(self) -> str:
            return "fake assertion"

        @override
        def _get_assertions(  # type: ignore[override]
            self, name: str | None = None
        ) -> list[FakeAssertion]:
            return [
                FakeAssertion(test_field_1="test-value-1", test_field_2=0),
                FakeAssertion(test_field_1="test-value-2", test_field_2=100),
            ]

        @override
        def _normalize_assertions(  # type: ignore[override]
            self, assertions: list[FakeAssertion]
        ) -> tuple[list[str], list[list[Any]]]:
            headers = ["test-field-1", "test-field-2"]
            assertion_data = [
                [
                    assertion.test_field_1,
                    assertion.test_field_2,
                ]
                for assertion in assertions
            ]
            return headers, assertion_data

    return FakeAssertionService(app=APP_METADATA, services=default_factory)


def test_list_assertions_table(fake_assertion_service, emitter):
    """List assertions as a table."""
    fake_assertion_service.list_assertions(
        output_format=const.OutputFormat.table, name="test-registry"
    )

    emitter.assert_message(
        textwrap.dedent(
            """\
            test-field-1      test-field-2
            test-value-1                 0
            test-value-2               100"""
        )
    )


def test_list_assertions_json(fake_assertion_service, emitter):
    """List assertions as json."""
    fake_assertion_service.list_assertions(
        output_format=const.OutputFormat.json, name="test-registry"
    )

    emitter.assert_message(
        textwrap.dedent(
            """\
            {
                "fake assertions": [
                    {
                        "test-field-1": "test-value-1",
                        "test-field-2": 0
                    },
                    {
                        "test-field-1": "test-value-2",
                        "test-field-2": 100
                    }
                ]
            }"""
        )
    )


def test_list_assertions_unknown_format(fake_assertion_service):
    """Error for unknown formats."""
    expected = "Command or feature not implemented: '--format unknown'"

    with pytest.raises(errors.FeatureNotImplemented, match=expected):
        fake_assertion_service.list_assertions(
            output_format="unknown", name="test-registry"
        )
