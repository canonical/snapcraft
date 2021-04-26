# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

import json
from typing import Dict, Any
from unittest import mock

import pytest

from snapcraft.storeapi.v2 import validation_sets
from snapcraft.storeapi import StoreClient


@pytest.fixture
def validation_sets_payload():
    return validation_sets.ValidationSets.unmarshal(
        {
            "assertions": [
                {
                    "headers": {
                        "account-id": "AccountIDXXXOfTheRequestingUserX",
                        "authority-id": "AccountIDXXXOfTheRequestingUserX",
                        "name": "certification-x1",
                        "revision": "222",
                        "sequence": "9",
                        "series": "16",
                        "sign-key-sha3-384": "XSignXKeyXHashXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
                        "snaps": [
                            {
                                "id": "XXSnapIDForXSnapName1XXXXXXXXXXX",
                                "name": "snap-name-1",
                                "presence": "optional",
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
    )


@pytest.fixture(autouse=True)
def fake_dashboard_post_validation_sets(validation_sets_payload):
    patched_get_validation_sets = mock.patch.object(
        StoreClient, "post_validation_sets", return_value=validation_sets_payload
    )
    yield patched_get_validation_sets.start()
    patched_get_validation_sets.stop()


@pytest.fixture(autouse=True)
def fake_dashboard_post_validation_sets_build_assertion(validation_sets_payload):
    patched_get_validation_sets = mock.patch.object(
        StoreClient,
        "post_validation_sets_build_assertion",
        return_value=validation_sets_payload.assertions[0],
    )
    yield patched_get_validation_sets.start()
    patched_get_validation_sets.stop()


@pytest.fixture(autouse=True)
def fake_dashboard_get_validation_sets(validation_sets_payload):
    patched_get_validation_sets = mock.patch.object(
        StoreClient, "get_validation_sets", return_value=validation_sets_payload
    )
    yield patched_get_validation_sets.start()
    patched_get_validation_sets.stop()


@pytest.fixture(autouse=True)
def fake_snap_sign():
    def sign(assertion: Dict[str, Any], *, key_name: str) -> bytes:
        return (json.dumps(assertion) + f"\n\nSIGNED{key_name}").encode()

    patched_snap_sign = mock.patch(
        "snapcraft.cli.assertions._sign_assertion", side_effect=sign
    )
    yield patched_snap_sign.start()
    patched_snap_sign.stop()


@pytest.mark.usefixtures("mock_subprocess_run")
def test_edit_validation_sets_with_no_changes_to_existing_set(
    click_run,
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
):
    cmd = [
        "edit-validation-sets",
        "AccountIDXXXOfTheRequestingUserX",
        "certification-x1",
        "9",
    ]

    result = click_run(cmd)

    assert result.exit_code == 0
    assert result.output.strip() == "No changes made."
    fake_dashboard_get_validation_sets.assert_called_once_with(
        name="certification-x1", sequence="9"
    )
    fake_dashboard_post_validation_sets_build_assertion.assert_not_called()
    fake_snap_sign.assert_not_called()
    fake_dashboard_post_validation_sets.assert_not_called()


@pytest.fixture
def fake_edit_validation_sets():
    patched_edit_validation_sets = mock.patch(
        "snapcraft.cli.assertions._edit_validation_sets"
    )
    yield patched_edit_validation_sets.start()
    patched_edit_validation_sets.stop()


@pytest.mark.parametrize("key_name", [None, "general", "main"])
def test_edit_validation_sets_with_changes_to_existing_set(
    click_run,
    validation_sets_payload,
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
    fake_edit_validation_sets,
    key_name,
):
    cmd = [
        "edit-validation-sets",
        "AccountIDXXXOfTheRequestingUserX",
        "certification-x1",
        "9",
    ]
    if key_name is not None:
        cmd.extend(["--key-name", key_name])

    fake_edit_validation_sets.return_value = {
        "account-id": "AccountIDXXXOfTheRequestingUserX",
        "name": "certification-x1",
        "sequence": "9",
        "snaps": [
            {
                "name": "snap-name-1",
                "id": "XXSnapIDForXSnapName1XXXXXXXXXXX",
                "presence": "required",
            },
            {"name": "snap-name-2", "id": "XXSnapIDForXSnapName2XXXXXXXXXXX"},
        ],
    }

    result = click_run(cmd)

    fake_dashboard_get_validation_sets.assert_called_once_with(
        name="certification-x1", sequence="9"
    )
    fake_dashboard_post_validation_sets_build_assertion.assert_called_once_with(
        validation_sets={
            "account-id": "AccountIDXXXOfTheRequestingUserX",
            "name": "certification-x1",
            "sequence": "9",
            "snaps": [
                {
                    "name": "snap-name-1",
                    "id": "XXSnapIDForXSnapName1XXXXXXXXXXX",
                    "presence": "required",
                },
                {"name": "snap-name-2", "id": "XXSnapIDForXSnapName2XXXXXXXXXXX"},
            ],
        }
    )
    fake_dashboard_post_validation_sets.assert_called_once_with(
        signed_validation_sets=(
            '{"account-id": "AccountIDXXXOfTheRequestingUserX", "authority-id": '
            '"AccountIDXXXOfTheRequestingUserX", "name": "certification-x1", '
            '"revision": "222", "sequence": "9", "snaps": [{"name": "snap-name-1", '
            '"id": "XXSnapIDForXSnapName1XXXXXXXXXXX", "presence": "optional"}, '
            '{"name": "snap-name-2", "id": "XXSnapIDForXSnapName2XXXXXXXXXXX"}], '
            '"timestamp": "2020-10-29T16:36:56Z", "type": "validation-set", '
            f'"series": "16"}}\n\nSIGNED{key_name}'
        ).encode()
    )
    fake_snap_sign.assert_called_once_with(
        validation_sets_payload.assertions[0].marshal(), key_name=key_name,
    )
    assert result.exit_code == 0
    assert result.output.strip() == ""
