# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

import argparse
import json
from typing import Any, Dict
from unittest.mock import call

import pytest
import requests

from snapcraft import commands, errors
from snapcraft.commands.validation_sets import StoreClientCLI, edit_validation_sets
from snapcraft_legacy.storeapi.errors import StoreValidationSetsError
from snapcraft_legacy.storeapi.v2 import validation_sets
from tests.unit.store.utils import FakeResponse

############
# Fixtures #
############

VALIDATION_SET_DATA = {
    "assertions": [
        {
            "headers": {
                "account-id": "AccountIDXXXOfTheRequestingUserX",
                "authority-id": "AccountIDXXXOfTheRequestingUserX",
                "name": "certification-x1",
                "revision": "222",
                "sequence": "9",
                "series": "16",
                "sign-key-sha3-384": "XSignXKeyXHashXXXXXXXXXXX",
                "snaps": [
                    {
                        "id": "XXSnapIDForXSnapName1XXXXXXXXXXX",
                        "name": "snap-name-1",
                        "presence": "optional",
                        "revision": "10",
                    },
                    {
                        "id": "XXSnapIDForXSnapName2XXXXXXXXXXX",
                        "name": "snap-name-2",
                    },
                ],
                "timestamp": "2020-10-29T16:36:56Z",
                "type": "validation-set",
            }
        },
    ]
}


@pytest.fixture
def fake_validation_sets():
    return validation_sets.ValidationSets.unmarshal(VALIDATION_SET_DATA)


@pytest.fixture()
def fake_build_assertion():
    return validation_sets.BuildAssertion.unmarshal(
        VALIDATION_SET_DATA["assertions"][0]["headers"]
    )


@pytest.fixture
def fake_dashboard_post_validation_sets(fake_validation_sets, mocker):
    return mocker.patch.object(
        StoreClientCLI, "post_validation_sets", return_value=fake_validation_sets
    )


@pytest.fixture
def fake_dashboard_post_validation_sets_build_assertion(fake_validation_sets, mocker):
    return mocker.patch.object(
        StoreClientCLI,
        "post_validation_sets_build_assertion",
        return_value=validation_sets.BuildAssertion.unmarshal(
            VALIDATION_SET_DATA["assertions"][0]["headers"]
        ),
    )


@pytest.fixture
def fake_dashboard_get_validation_sets(fake_validation_sets, mocker):
    return mocker.patch.object(
        StoreClientCLI, "get_validation_sets", return_value=fake_validation_sets
    )


@pytest.fixture
def fake_snap_sign(mocker):
    def sign(assertion: Dict[str, Any], *, key_name: str) -> bytes:
        return (json.dumps(assertion) + f"\n\nSIGNED{key_name}").encode()

    return mocker.patch(
        "snapcraft.commands.validation_sets._sign_assertion", side_effect=sign
    )


@pytest.fixture
def edit_return_value():
    return validation_sets.EditableBuildAssertion.unmarshal(
        {
            "account-id": "AccountIDXXXOfTheRequestingUserX",
            "name": "certification-x1",
            "sequence": "9",
            "snaps": [
                {
                    "id": "XXSnapIDForXSnapName1XXXXXXXXXXX",
                    "name": "snap-name-1",
                    "presence": "required",
                    "revision": "10",
                },
                {"id": "XXSnapIDForXSnapName2XXXXXXXXXXX", "name": "snap-name-2"},
            ],
        }
    )


@pytest.fixture
def fake_edit_validation_sets(mocker, edit_return_value):
    """A fake for editing validation sets that returns no changes by default."""
    return mocker.patch(
        "snapcraft.commands.validation_sets.edit_validation_sets",
        return_value=edit_return_value,
    )


##################
# Expected Calls #
##################

EXPECTED_GET_VALIDATION_SETS_CALL = call(name="certification-x1", sequence="9")
EXPECTED_BUILD_ASSERTION_CALL = call(
    validation_sets={
        "account-id": "AccountIDXXXOfTheRequestingUserX",
        "name": "certification-x1",
        "sequence": 9,
        "snaps": [
            {
                "name": "snap-name-1",
                "id": "XXSnapIDForXSnapName1XXXXXXXXXXX",
                "presence": "required",
                "revision": 10,
            },
            {"name": "snap-name-2", "id": "XXSnapIDForXSnapName2XXXXXXXXXXX"},
        ],
    }
)

EXPECTED_SIGNED_VALIDATION_SETS = (
    '{{"account-id": "AccountIDXXXOfTheRequestingUserX", "name": "certification-x1", '
    '"revision": "222", "sequence": "9", "snaps": [{{"name": "snap-name-1", "id": '
    '"XXSnapIDForXSnapName1XXXXXXXXXXX", "presence": "optional", "revision": "10"}}, '
    '{{"name": "snap-name-2", "id": "XXSnapIDForXSnapName2XXXXXXXXXXX"}}], '
    '"authority-id": "AccountIDXXXOfTheRequestingUserX", "series": "16", "timestamp": '
    '"2020-10-29T16:36:56Z", "type": "validation-set"}}\n\nSIGNED{key_name}'
)


################################
# Edit Validation Sets Command #
################################


@pytest.mark.usefixtures("memory_keyring")
def test_edit_validation_sets_with_no_changes_to_existing_set(
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
    edit_return_value,
    fake_edit_validation_sets,
    emitter,
):
    # Make it look like there were no edits.
    edit_return_value.sequence = 9
    edit_return_value.snaps[0].presence = "optional"
    fake_edit_validation_sets.return_value = edit_return_value

    cmd = commands.StoreEditValidationSetsCommand(None)

    cmd.run(
        argparse.Namespace(
            account_id="AccountIDXXXOfTheRequestingUserX",
            set_name="certification-x1",
            sequence="9",
            key_name=None,
        )
    )

    emitter.assert_message("No changes made")
    assert fake_dashboard_get_validation_sets.mock_calls == [
        EXPECTED_GET_VALIDATION_SETS_CALL
    ]
    assert fake_dashboard_post_validation_sets_build_assertion.mock_calls == []
    assert fake_snap_sign.mock_calls == []
    assert fake_dashboard_post_validation_sets.mock_calls == []


@pytest.mark.usefixtures("memory_keyring", "fake_edit_validation_sets")
@pytest.mark.parametrize("key_name", [None, "general", "main"])
def test_edit_validation_sets_with_changes_to_existing_set(
    fake_validation_sets,
    fake_build_assertion,
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
    key_name,
):
    cmd = commands.StoreEditValidationSetsCommand(None)

    cmd.run(
        argparse.Namespace(
            account_id="AccountIDXXXOfTheRequestingUserX",
            set_name="certification-x1",
            sequence="9",
            key_name=key_name,
        )
    )

    assert fake_dashboard_get_validation_sets.mock_calls == [
        EXPECTED_GET_VALIDATION_SETS_CALL
    ]
    assert fake_dashboard_post_validation_sets_build_assertion.mock_calls == [
        EXPECTED_BUILD_ASSERTION_CALL
    ]
    assert fake_dashboard_post_validation_sets.mock_calls == [
        call(
            signed_validation_sets=EXPECTED_SIGNED_VALIDATION_SETS.format(
                key_name=key_name
            ).encode()
        ),
    ]
    assert fake_snap_sign.mock_calls == [
        call(
            fake_build_assertion.marshal_scalars_as_strings(),
            key_name=key_name,
        )
    ]


@pytest.mark.usefixtures("memory_keyring", "fake_edit_validation_sets")
def test_edit_validation_sets_with_errors_to_amend(
    fake_validation_sets,
    fake_build_assertion,
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
    mocker,
):
    fake_dashboard_post_validation_sets_build_assertion.side_effect = [
        StoreValidationSetsError(
            FakeResponse(
                status_code=requests.codes.bad_request,
                content=json.dumps(
                    {"error_list": [{"message": "bad assertion", "code": "no snap"}]}
                ).encode(),
            )
        ),
        fake_build_assertion,
    ]
    confirm_mock = mocker.patch("snapcraft.utils.confirm_with_user", return_value=True)

    cmd = commands.StoreEditValidationSetsCommand(None)

    cmd.run(
        argparse.Namespace(
            account_id="AccountIDXXXOfTheRequestingUserX",
            set_name="certification-x1",
            sequence="9",
            key_name=None,
        )
    )

    assert fake_dashboard_get_validation_sets.mock_calls == [
        EXPECTED_GET_VALIDATION_SETS_CALL
    ]
    assert fake_dashboard_post_validation_sets_build_assertion.mock_calls == [
        EXPECTED_BUILD_ASSERTION_CALL,
        EXPECTED_BUILD_ASSERTION_CALL,
    ]
    assert fake_dashboard_post_validation_sets.mock_calls == [
        call(
            signed_validation_sets=EXPECTED_SIGNED_VALIDATION_SETS.format(
                key_name=None
            ).encode()
        )
    ]
    assert fake_snap_sign.mock_calls == [
        call(
            fake_build_assertion.marshal_scalars_as_strings(),
            key_name=None,
        )
    ]
    assert confirm_mock.mock_calls == [call("Do you wish to amend the validation set?")]


@pytest.mark.usefixtures("memory_keyring", "fake_edit_validation_sets")
def test_edit_validation_sets_with_errors_not_amended(
    fake_validation_sets,
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
    mocker,
):
    fake_dashboard_post_validation_sets_build_assertion.side_effect = (
        StoreValidationSetsError(
            FakeResponse(
                status_code=requests.codes.bad_request,
                content=json.dumps(
                    {"error_list": [{"message": "bad assertion", "code": "no snap"}]}
                ).encode(),
            )
        )
    )
    confirm_mock = mocker.patch("snapcraft.utils.confirm_with_user", return_value=False)

    cmd = commands.StoreEditValidationSetsCommand(None)

    with pytest.raises(errors.SnapcraftError):
        cmd.run(
            argparse.Namespace(
                account_id="AccountIDXXXOfTheRequestingUserX",
                set_name="certification-x1",
                sequence="9",
                key_name=None,
            )
        )

    assert fake_dashboard_get_validation_sets.mock_calls == [
        EXPECTED_GET_VALIDATION_SETS_CALL
    ]
    assert fake_dashboard_post_validation_sets_build_assertion.mock_calls == [
        EXPECTED_BUILD_ASSERTION_CALL
    ]
    assert fake_dashboard_post_validation_sets.mock_calls == []
    assert fake_snap_sign.mock_calls == []
    assert confirm_mock.mock_calls == [call("Do you wish to amend the validation set?")]


def test_edit_yaml_error_retry(mocker, tmp_path, monkeypatch):
    monkeypatch.setenv("EDITOR", "faux-vi")
    tmp_file = tmp_path / "validation_sets_template"
    confirm_mock = mocker.patch("snapcraft.utils.confirm_with_user", return_value=True)
    expected_edited_assertion = validation_sets.EditableBuildAssertion.unmarshal(
        {
            "account-id": "test",
            "name": "test",
            "sequence": 1,
            "snaps": [{"name": "core22"}],
        }
    )
    good_yaml = "{'account-id': 'test', 'name': 'test', 'sequence': 1, 'snaps': [{'name': 'core22'}]}"
    bad_yaml = f"badyaml}} {good_yaml}"
    data_write = [good_yaml, bad_yaml]

    def side_effect(*args, **kwargs):
        tmp_file.write_text(data_write.pop(), encoding="utf-8")

    subprocess_mock = mocker.patch("subprocess.run", side_effect=side_effect)

    assert edit_validation_sets(tmp_file) == expected_edited_assertion
    assert confirm_mock.mock_calls == [call("Do you wish to amend the validation set?")]
    assert subprocess_mock.mock_calls == [call(["faux-vi", tmp_file], check=True)] * 2


def test_edit_pydantic_error_retry(mocker, tmp_path, monkeypatch):
    monkeypatch.setenv("EDITOR", "faux-vi")
    tmp_file = tmp_path / "validation_sets_template"
    confirm_mock = mocker.patch("snapcraft.utils.confirm_with_user", return_value=True)
    expected_edited_assertion = validation_sets.EditableBuildAssertion.unmarshal(
        {
            "account-id": "test",
            "name": "test",
            "sequence": 1,
            "snaps": [{"name": "core22"}],
        }
    )
    good_yaml = "{'account-id': 'test', 'name': 'test', 'sequence': 1, 'snaps': [{'name': 'core22'}]}"
    bad_yaml = "{'invalid-field': 'test'}"
    data_write = [good_yaml, bad_yaml]

    def side_effect(*args, **kwargs):
        tmp_file.write_text(data_write.pop(), encoding="utf-8")

    subprocess_mock = mocker.patch("subprocess.run", side_effect=side_effect)

    assert edit_validation_sets(tmp_file) == expected_edited_assertion
    assert confirm_mock.mock_calls == [call("Do you wish to amend the validation set?")]
    assert subprocess_mock.mock_calls == [call(["faux-vi", tmp_file], check=True)] * 2


def test_edit_yaml_error_no_retry(mocker, tmp_path, monkeypatch):
    monkeypatch.setenv("EDITOR", "faux-vi")

    tmp_file = tmp_path / "validation_sets_template"
    confirm_mock = mocker.patch("snapcraft.utils.confirm_with_user", return_value=False)

    def side_effect(*args, **kwargs):
        tmp_file.write_text("{{bad yaml {{", encoding="utf-8")

    subprocess_mock = mocker.patch("subprocess.run", side_effect=side_effect)

    with pytest.raises(errors.SnapcraftError):
        edit_validation_sets(tmp_file)

    assert confirm_mock.mock_calls == [call("Do you wish to amend the validation set?")]
    assert subprocess_mock.mock_calls == [call(["faux-vi", tmp_file], check=True)]
