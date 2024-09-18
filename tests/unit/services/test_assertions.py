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
from unittest import mock

import pytest
from craft_application.models import CraftBaseModel
from typing_extensions import override

from snapcraft import const, errors


@pytest.fixture(autouse=True)
def mock_store_client(mocker):
    """Mock the store client before it is initialized in the service's setup."""
    return mocker.patch(
        "snapcraft.store.StoreClientCLI",
    )


@pytest.fixture(autouse=True)
def fake_editor(monkeypatch):
    """Set a fake editor."""
    return monkeypatch.setenv("EDITOR", "faux-vi")


@pytest.fixture
def mock_confirm_with_user(mocker, request):
    """Mock the confirm_with_user function."""
    return mocker.patch("snapcraft.utils.confirm_with_user", return_value=request.param)


@pytest.fixture
def mock_subprocess_run(mocker, tmp_path, request):
    """Mock the subprocess.run function to write data to a file.

    :param request: A list of strings to write to a file. Each time the subprocess.run
      function is called, the last string in the list will be written to the file
      and removed from the list.
    """
    data_write = request.param.copy()
    tmp_file = tmp_path / "assertion-file"

    def side_effect(*args, **kwargs):
        tmp_file.write_text(data_write.pop(), encoding="utf-8")

    subprocess_mock = mocker.patch("subprocess.run", side_effect=side_effect)
    return subprocess_mock


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

        @property
        @override
        def _editable_assertion_class(  # type: ignore[override]
            self,
        ) -> type[FakeAssertion]:
            return FakeAssertion

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

        @override
        def _generate_yaml_from_model(  # type: ignore[override]
            self, assertion: FakeAssertion
        ) -> str:
            return textwrap.dedent(
                """\
                test-field-1: test-value-1
                test-field-2: 0
                """
            )

        @override
        def _generate_yaml_from_template(self, name: str, account_id: str) -> str:
            return textwrap.dedent(
                """\
                test-field-1: default-value-1
                test-field-2: 0
                """
            )

    return FakeAssertionService(app=APP_METADATA, services=default_factory)


@pytest.fixture
def fake_edit_yaml_file(mocker, fake_assertion_service):
    """Apply a fake edit to a yaml file."""
    return mocker.patch.object(
        fake_assertion_service,
        "_edit_yaml_file",
        return_value=FakeAssertion(
            test_field_1="test-value-1-UPDATED", test_field_2=999
        ),
    )


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


def test_edit_assertions_changes_made(
    fake_edit_yaml_file, fake_assertion_service, emitter
):
    """Edit an assertion and make a valid change."""
    expected = "Building, signing and uploading fake assertion is not implemented"
    fake_assertion_service.setup()

    with pytest.raises(errors.FeatureNotImplemented, match=expected):
        fake_assertion_service.edit_assertion(
            name="test-registry", account_id="test-account-id"
        )

    emitter.assert_message("Successfully edited fake assertion 'test-registry'.")


def test_edit_assertions_no_changes_made(
    fake_edit_yaml_file, fake_assertion_service, emitter, mocker
):
    """Edit an assertion but make no changes to the data."""
    mocker.patch.object(
        fake_assertion_service,
        "_edit_yaml_file",
        # make no changes to the fake assertion
        return_value=FakeAssertion(test_field_1="test-value-1", test_field_2=0),
    )
    fake_assertion_service.setup()

    fake_assertion_service.edit_assertion(
        name="test-registry", account_id="test-account-id"
    )

    emitter.assert_message("No changes made.")


@pytest.mark.parametrize("editor", [None, "faux-vi"])
@pytest.mark.parametrize(
    "mock_subprocess_run",
    [
        [
            textwrap.dedent(
                """\
                test-field-1: test-value-1-UPDATED
                test-field-2: 999
                """
            ),
        ],
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [True], indirect=True)
def test_edit_yaml_file(
    editor,
    fake_assertion_service,
    tmp_path,
    mock_confirm_with_user,
    mock_subprocess_run,
    monkeypatch,
):
    """Successfully edit a yaml file with the correct editor."""
    if editor:
        monkeypatch.setenv("EDITOR", editor)
        expected_editor = editor
    else:
        monkeypatch.delenv("EDITOR", raising=False)
        # default is vi
        expected_editor = "vi"
    tmp_file = tmp_path / "assertion-file"
    fake_assertion_service.setup()

    edited_assertion = fake_assertion_service._edit_yaml_file(tmp_file)

    assert edited_assertion == FakeAssertion(
        test_field_1="test-value-1-UPDATED", test_field_2=999
    )
    mock_confirm_with_user.assert_not_called()
    assert mock_subprocess_run.mock_calls == [
        mock.call([expected_editor, tmp_file], check=True)
    ]


@pytest.mark.parametrize(
    "mock_subprocess_run",
    [
        pytest.param(
            [
                textwrap.dedent(
                    """\
                    test-field-1: test-value-1-UPDATED
                    test-field-2: 999
                    """
                ),
                textwrap.dedent(
                    """\
                    bad yaml {{
                    test-field-1: test-value-1
                    test-field-2: 0
                    """
                ),
            ],
            id="invalid yaml syntax",
        ),
        pytest.param(
            [
                textwrap.dedent(
                    """\
                    test-field-1: test-value-1-UPDATED
                    test-field-2: 999
                    """
                ),
                textwrap.dedent(
                    """\
                    extra-field: not-allowed
                    test-field-1: [wrong data type]
                    test-field-2: 0
                    """
                ),
            ],
            id="invalid pydantic data",
        ),
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [True], indirect=True)
def test_edit_yaml_file_error_retry(
    fake_assertion_service,
    tmp_path,
    mock_confirm_with_user,
    mock_subprocess_run,
):
    """Edit a yaml file but encounter an error and retry."""
    tmp_file = tmp_path / "assertion-file"
    fake_assertion_service.setup()

    edited_assertion = fake_assertion_service._edit_yaml_file(tmp_file)

    assert edited_assertion == FakeAssertion(
        test_field_1="test-value-1-UPDATED", test_field_2=999
    )
    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert (
        mock_subprocess_run.mock_calls
        == [mock.call(["faux-vi", tmp_file], check=True)] * 2
    )


@pytest.mark.parametrize(
    "mock_subprocess_run",
    [
        [
            textwrap.dedent(
                """\
                bad yaml {{
                test-field-1: test-value-1
                test-field-2: 0
                """
            ),
        ],
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [False], indirect=True)
def test_edit_error_no_retry(
    fake_assertion_service,
    tmp_path,
    mock_confirm_with_user,
    mock_subprocess_run,
):
    """Edit a yaml file and encounter an error but do not retry."""
    tmp_file = tmp_path / "assertion-file"
    fake_assertion_service.setup()

    with pytest.raises(errors.SnapcraftError, match="operation aborted"):
        fake_assertion_service._edit_yaml_file(tmp_file)

    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert mock_subprocess_run.mock_calls == [
        mock.call(["faux-vi", tmp_file], check=True)
    ]
