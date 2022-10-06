# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

from snapcraft import commands
from snapcraft.commands.validation_sets import StoreClientCLI
from snapcraft_legacy.storeapi.v2 import validation_sets

############
# Fixtures #
############


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
                        "sign-key-sha3-384": "XSignXKeyXHashXXXXXXXXXXX",
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


@pytest.fixture
def fake_dashboard_post_validation_sets(validation_sets_payload, mocker):
    return mocker.patch.object(
        StoreClientCLI, "post_validation_sets", return_value=validation_sets_payload
    )


@pytest.fixture
def fake_dashboard_post_validation_sets_build_assertion(
    validation_sets_payload, mocker
):
    return mocker.patch.object(
        StoreClientCLI,
        "post_validation_sets_build_assertion",
        return_value=validation_sets_payload.assertions[0],
    )


@pytest.fixture
def fake_dashboard_get_validation_sets(validation_sets_payload, mocker):
    return mocker.patch.object(
        StoreClientCLI, "get_validation_sets", return_value=validation_sets_payload
    )


@pytest.fixture
def fake_snap_sign(mocker):
    def sign(assertion: Dict[str, Any], *, key_name: str) -> bytes:
        return (json.dumps(assertion) + f"\n\nSIGNED{key_name}").encode()

    return mocker.patch(
        "snapcraft.commands.validation_sets._sign_assertion", side_effect=sign
    )


@pytest.fixture
def fake_edit_validation_sets(mocker):
    """A fake for editing validation sets that returns no changes by default."""
    return mocker.patch(
        "snapcraft.commands.validation_sets._edit_validation_sets",
        return_value={
            "account-id": "AccountIDXXXOfTheRequestingUserX",
            "name": "certification-x1",
            "sequence": 9,
            "snaps": [
                {
                    "id": "XXSnapIDForXSnapName1XXXXXXXXXXX",
                    "name": "snap-name-1",
                    "presence": "optional",
                },
                {"id": "XXSnapIDForXSnapName2XXXXXXXXXXX", "name": "snap-name-2"},
            ],
        },
    )


################################
# Edit Validation Sets Command #
################################


@pytest.mark.usefixtures("memory_keyring")
@pytest.mark.usefixtures("fake_edit_validation_sets")
def test_edit_validation_sets_with_no_changes_to_existing_set(
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
    emitter,
):
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
        call(name="certification-x1", sequence="9")
    ]
    assert fake_dashboard_post_validation_sets_build_assertion.mock_calls == []
    assert fake_snap_sign.mock_calls == []
    assert fake_dashboard_post_validation_sets.mock_calls == []


@pytest.mark.usefixtures("memory_keyring")
@pytest.mark.parametrize("key_name", [None, "general", "main"])
def test_edit_validation_sets_with_changes_to_existing_set(
    validation_sets_payload,
    fake_dashboard_get_validation_sets,
    fake_dashboard_post_validation_sets_build_assertion,
    fake_dashboard_post_validation_sets,
    fake_snap_sign,
    fake_edit_validation_sets,
    key_name,
):
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
        call(name="certification-x1", sequence="9")
    ]
    assert fake_dashboard_post_validation_sets_build_assertion.mock_calls == [
        call(
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
    ]
    assert fake_dashboard_post_validation_sets.mock_calls == [
        call(
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
    ]
    assert fake_snap_sign.mock_calls == [
        call(
            validation_sets_payload.assertions[0].marshal(),
            key_name=key_name,
        )
    ]
