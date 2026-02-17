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

import json
import tempfile
import textwrap
from typing import Any
from unittest import mock

import craft_store.errors
import pytest
from craft_application.models import CraftBaseModel
from typing_extensions import override

from snapcraft import const, errors
from tests.unit.store.utils import FakeResponse


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
def write_text(mocker, tmp_path, request):
    """Mock the subprocess.run function to write fake data to a temp assertion file.

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


@pytest.fixture
def fake_sign_assertion(mocker):
    def _fake_sign(cmdline, input):  # noqa: A002 (builtin-argument-shadowing)
        return input + b"-signed"

    mock_subprocess = mocker.patch("subprocess.check_output")
    mock_subprocess.side_effect = _fake_sign
    return mock_subprocess


@pytest.fixture(autouse=True)
def mock_named_temporary_file(mocker, tmp_path):
    _mock_tempfile = mocker.patch(
        "tempfile.NamedTemporaryFile", spec=tempfile.NamedTemporaryFile
    )
    _mock_tempfile.return_value.__enter__.return_value.name = str(
        tmp_path / "assertion-file"
    )
    yield _mock_tempfile.return_value


FAKE_STORE_ERROR = craft_store.errors.StoreServerError(
    response=FakeResponse(
        content=json.dumps(
            {"error_list": [{"code": "bad assertion", "message": "bad assertion"}]}
        ),
        status_code=400,
    )
)


class FakeAssertion(CraftBaseModel):
    """Fake assertion model."""

    test_field_1: str
    test_field_2: int

    def marshal_scalars_as_strings(self):
        return {
            "test_field_1": self.test_field_1,
            "test_field_2": str(self.test_field_2),
        }


@pytest.fixture
def fake_assertion_service(fake_services):
    from snapcraft.application import (  # noqa: PLC0415 (import-outside-top-level)
        APP_METADATA,
    )
    from snapcraft.services import Assertion  # noqa: PLC0415 (import-outside-top-level)

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
            self, name: str | None = None, **kwargs: dict[str, Any]
        ) -> list[FakeAssertion]:
            return [
                FakeAssertion(test_field_1="test-value-1", test_field_2=0),
                FakeAssertion(test_field_1="test-value-2", test_field_2=100),
            ]

        @override
        def _build_assertion(  # type: ignore[override]
            self, assertion: FakeAssertion
        ) -> FakeAssertion:
            assertion.test_field_1 = assertion.test_field_1 + "-built"
            return assertion

        @override
        def _post_assertion(  # type: ignore[override]
            self, assertion_data: bytes
        ) -> FakeAssertion:
            return FakeAssertion(
                test_field_1="test-published-assertion", test_field_2=0
            )

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
        def _generate_yaml_from_template(
            self, name: str, account_id: str, **kwargs: dict[str, Any]
        ) -> str:
            return textwrap.dedent(
                """\
                test-field-1: default-value-1
                test-field-2: 0
                """
            )

        @override
        def _get_success_message(  # type: ignore[override]
            self, assertion: FakeAssertion
        ) -> str:
            return "Success."

        @override
        def _validate_assertion(  # type: ignore[override]
            self,
            assertion: FakeAssertion,
            **kwargs: dict[str, Any],
        ) -> None:
            """Add an simple custom validator."""
            if kwargs.get("test-arg") == assertion.test_field_2:
                raise errors.SnapcraftAssertionError("Custom validation failed.")

    return FakeAssertionService(app=APP_METADATA, services=fake_services)


def test_list_assertions_table(fake_assertion_service, emitter):
    """List assertions as a table."""
    fake_assertion_service.list_assertions(
        output_format=const.OutputFormat.table, name="test-confb"
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
        output_format=const.OutputFormat.json, name="test-confb"
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
            output_format="unknown", name="test-confb"
        )


@pytest.mark.parametrize(
    "write_text",
    [["test-field-1: test-value-1-edited\ntest-field-2: 999"]],
    indirect=True,
)
@pytest.mark.usefixtures("fake_sign_assertion")
def test_edit_assertions_changes_made(
    fake_assertion_service,
    emitter,
    mocker,
    tmp_path,
    write_text,
):
    """Edit an assertion and make a valid change."""
    expected_assertion = (
        b'{"test_field_1": "test-value-1-edited-built", "test_field_2": "999"}-signed'
    )
    mock_post_assertion = mocker.spy(fake_assertion_service, "_post_assertion")

    fake_assertion_service.setup()
    fake_assertion_service.edit_assertion(
        name="test-confb", account_id="test-account-id", key_name="test-key"
    )

    mock_post_assertion.assert_called_once_with(expected_assertion)
    emitter.assert_trace(f"Signed assertion: {expected_assertion.decode()}")
    emitter.assert_message("Success.")


@pytest.mark.parametrize(
    "write_text",
    [["test-field-1: test-value-1\ntest-field-2: 0"]],
    indirect=True,
)
def test_edit_assertions_no_changes_made(
    fake_assertion_service, emitter, tmp_path, write_text
):
    """Edit an assertion but make no changes to the data."""
    fake_assertion_service.setup()
    fake_assertion_service.edit_assertion(
        name="test-confb", account_id="test-account-id"
    )

    emitter.assert_message("No changes made.")
    assert not (tmp_path / "assertion-file").exists()


@pytest.mark.parametrize(
    "write_text",
    [
        [
            "test-field-1: test-value-1-edited-edited\ntest-field-2: 999",
            "test-field-1: test-value-1-edited\ntest-field-2: 999",
        ],
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [True], indirect=True)
@pytest.mark.parametrize(
    "error", [FAKE_STORE_ERROR, errors.SnapcraftAssertionError("bad assertion")]
)
@pytest.mark.usefixtures("fake_sign_assertion")
def test_edit_assertions_build_assertion_error(
    error,
    fake_assertion_service,
    emitter,
    mock_confirm_with_user,
    write_text,
    mocker,
    tmp_path,
):
    """Receive an error while building an assertion, then re-edit and post the assertion."""
    expected_assertion = b'{"test_field_1": "test-value-1-edited-edited-built", "test_field_2": "999"}-signed'
    mock_post_assertion = mocker.spy(fake_assertion_service, "_post_assertion")
    mocker.patch.object(
        fake_assertion_service,
        "_build_assertion",
        side_effect=[
            error,
            FakeAssertion(
                test_field_1="test-value-1-edited-edited-built", test_field_2=999
            ),
        ],
    )

    fake_assertion_service.setup()
    fake_assertion_service.edit_assertion(
        name="test-confb", account_id="test-account-id", key_name="test-key"
    )

    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert mock_post_assertion.mock_calls == [mock.call(expected_assertion)]
    emitter.assert_trace(f"Signed assertion: {expected_assertion.decode()}")
    emitter.assert_message("Success.")
    assert not (tmp_path / "assertion-file").exists()


@pytest.mark.parametrize(
    "write_text",
    [
        [
            "test-field-1: test-value-1-edited-edited\ntest-field-2: 999",
            "test-field-1: test-value-1-edited\ntest-field-2: 999",
        ],
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [True], indirect=True)
@pytest.mark.usefixtures("fake_sign_assertion")
def test_edit_assertions_sign_assertion_error(
    fake_assertion_service,
    emitter,
    mock_confirm_with_user,
    write_text,
    mocker,
    tmp_path,
):
    """Receive an error while signing an assertion, then re-edit and post the assertion."""
    expected_assertion = b'{"test_field_1": "test-value-1-edited-edited-built", "test_field_2": "999"}-signed'
    mock_post_assertion = mocker.spy(fake_assertion_service, "_post_assertion")
    mocker.patch.object(
        fake_assertion_service,
        "_sign_assertion",
        side_effect=[
            errors.SnapcraftAssertionError("bad assertion"),
            expected_assertion,
        ],
    )

    fake_assertion_service.setup()
    fake_assertion_service.edit_assertion(
        name="test-confb", account_id="test-account-id", key_name="test-key"
    )

    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert mock_post_assertion.mock_calls == [mock.call(expected_assertion)]
    emitter.assert_message("Success.")
    assert not (tmp_path / "assertion-file").exists()


@pytest.mark.parametrize(
    "write_text",
    [
        [
            "test-field-1: test-value-1-edited-edited\ntest-field-2: 999",
            "test-field-1: test-value-1-edited\ntest-field-2: 999",
        ],
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [True], indirect=True)
@pytest.mark.parametrize(
    "error", [FAKE_STORE_ERROR, errors.SnapcraftAssertionError("bad assertion")]
)
@pytest.mark.usefixtures("fake_sign_assertion")
def test_edit_assertions_post_assertion_error(
    error,
    fake_assertion_service,
    emitter,
    mock_confirm_with_user,
    write_text,
    mocker,
    tmp_path,
):
    """Receive an error while processing an assertion, then re-edit and post the assertion."""
    expected_first_assertion = (
        b'{"test_field_1": "test-value-1-edited-built", "test_field_2": "999"}-signed'
    )
    expected_second_assertion = b'{"test_field_1": "test-value-1-edited-edited-built", "test_field_2": "999"}-signed'
    mock_post_assertion = mocker.patch.object(
        fake_assertion_service, "_post_assertion", side_effect=[error, None]
    )

    fake_assertion_service.setup()
    fake_assertion_service.edit_assertion(
        name="test-confb", account_id="test-account-id", key_name="test-key"
    )

    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert mock_post_assertion.mock_calls == [
        mock.call(expected_first_assertion),
        mock.call(expected_second_assertion),
    ]
    emitter.assert_trace(f"Signed assertion: {expected_second_assertion.decode()}")
    emitter.assert_message("Success.")
    assert not (tmp_path / "assertion-file").exists()


@pytest.mark.parametrize("editor", [None, "faux-vi"])
@pytest.mark.parametrize(
    "write_text",
    [["test-field-1: test-value-1-edited\ntest-field-2: 999"]],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [True], indirect=True)
def test_edit_yaml_file(
    editor,
    fake_assertion_service,
    tmp_path,
    mock_confirm_with_user,
    write_text,
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
        test_field_1="test-value-1-edited", test_field_2=999
    )
    mock_confirm_with_user.assert_not_called()
    assert write_text.mock_calls == [mock.call([expected_editor, tmp_file], check=True)]


@pytest.mark.parametrize(
    "write_text",
    [
        pytest.param(
            [
                "test-field-1: test-value-1-edited\ntest-field-2: 999",
                "bad yaml {{\ntest-field-1: test-value-1\ntest-field-2: 0",
            ],
            id="invalid yaml syntax",
        ),
        pytest.param(
            [
                "test-field-1: test-value-1-edited\ntest-field-2: 999",
                "extra-field: not-allowed\ntest-field-1: [wrong data type]\ntest-field-2: 0",
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
    write_text,
):
    """Edit a yaml file but encounter an error and retry."""
    tmp_file = tmp_path / "assertion-file"
    fake_assertion_service.setup()

    edited_assertion = fake_assertion_service._edit_yaml_file(tmp_file)

    assert edited_assertion == FakeAssertion(
        test_field_1="test-value-1-edited", test_field_2=999
    )
    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert write_text.mock_calls == [mock.call(["faux-vi", tmp_file], check=True)] * 2


@pytest.mark.parametrize(
    "write_text",
    [["bad yaml {{\ntest-field-1: test-value-1\ntest-field-2: 0"]],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [False], indirect=True)
def test_edit_error_no_retry(
    fake_assertion_service,
    tmp_path,
    mock_confirm_with_user,
    write_text,
):
    """Edit a yaml file and encounter an error but do not retry."""
    tmp_file = tmp_path / "assertion-file"
    fake_assertion_service.setup()

    with pytest.raises(errors.SnapcraftError, match="operation aborted"):
        fake_assertion_service._edit_yaml_file(tmp_file)

    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert write_text.mock_calls == [mock.call(["faux-vi", tmp_file], check=True)]


@pytest.mark.parametrize(
    "write_text",
    [
        [
            "test-field-1: test-value-1-edited\ntest-field-2: 1000",
            "test-field-1: test-value-1-edited\ntest-field-2: 999",
        ],
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [True], indirect=True)
@pytest.mark.usefixtures("fake_sign_assertion")
def test_edit_assertions_validate_assertion_error_retry(
    fake_assertion_service,
    emitter,
    mock_confirm_with_user,
    write_text,
    mocker,
    tmp_path,
):
    """Receive an error when validating an assertion, the re-edit and post the assertion."""
    # this will trigger an error in FakeAssertion.validate_assertion
    kwargs: dict[str, Any] = {"test-arg": 999}
    expected_assertion = (
        b'{"test_field_1": "test-value-1-edited-built", "test_field_2": "1000"}-signed'
    )
    mock_post_assertion = mocker.spy(fake_assertion_service, "_post_assertion")

    fake_assertion_service.setup()
    fake_assertion_service.edit_assertion(
        name="test-confb", account_id="test-account-id", key_name="test-key", **kwargs
    )

    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert mock_post_assertion.mock_calls == [mock.call(expected_assertion)]
    emitter.assert_trace(f"Signed assertion: {expected_assertion.decode()}")
    emitter.assert_message("Success.")
    assert not (tmp_path / "assertion-file").exists()


@pytest.mark.parametrize(
    "write_text",
    [
        [
            "test-field-1: test-value-1-edited\ntest-field-2: 999",
        ],
    ],
    indirect=True,
)
@pytest.mark.parametrize("mock_confirm_with_user", [False], indirect=True)
@pytest.mark.usefixtures("fake_sign_assertion")
def test_edit_assertions_validate_assertion_error(
    fake_assertion_service,
    emitter,
    mock_confirm_with_user,
    write_text,
    mocker,
    tmp_path,
):
    """Receive an error when validating an assertion and don't retry."""
    # this will trigger an error in FakeAssertion.validate_assertion
    kwargs: dict[str, Any] = {"test-arg": 999}
    mock_post_assertion = mocker.spy(fake_assertion_service, "_post_assertion")

    fake_assertion_service.setup()

    with pytest.raises(errors.SnapcraftError, match="operation aborted"):
        fake_assertion_service.edit_assertion(
            name="test-confb",
            account_id="test-account-id",
            key_name="test-key",
            **kwargs,
        )

    assert mock_confirm_with_user.mock_calls == [
        mock.call("Do you wish to amend the fake assertion?")
    ]
    assert mock_post_assertion.mock_calls == []
    assert not (tmp_path / "assertion-file").exists()
