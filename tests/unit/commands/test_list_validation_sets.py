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

from textwrap import dedent
from unittest import mock

import pytest

from snapcraft.storeapi.v2 import validation_sets
from snapcraft.storeapi import StoreClient


@pytest.fixture
def fake_dashboard_get_validation_sets():
    patched_get_validation_sets = mock.patch.object(
        StoreClient,
        "get_validation_sets",
        return_value=validation_sets.ValidationSets.unmarshal({"assertions": []}),
    )
    yield patched_get_validation_sets.start()
    patched_get_validation_sets.stop()


combinations = [
    {
        "name": None,
        "sequence": None,
        "output": "No validation sets found for this account.",
    },
    {
        "name": "foo",
        "sequence": None,
        "output": "No validation sets found for the requested name or sequence.",
    },
    {
        "name": "foo",
        "sequence": "all",
        "output": "No validation sets found for the requested name or sequence.",
    },
    {
        "name": None,
        "sequence": "all",
        "output": "No validation sets found for the requested name or sequence.",
    },
]


@pytest.mark.parametrize("combo,", combinations)
def test_no_sets(click_run, fake_dashboard_get_validation_sets, combo):
    cmd = ["list-validation-sets"]
    if combo["name"] is not None:
        cmd.extend(["--name", combo["name"]])
    if combo["sequence"] is not None:
        cmd.extend(["--sequence", combo["sequence"]])

    result = click_run(cmd)

    assert result.exit_code == 0
    assert result.output.strip() == combo["output"]
    fake_dashboard_get_validation_sets.assert_called_once_with(
        name=combo["name"], sequence=combo["sequence"]
    )


def test_list_validation_sets(click_run, fake_dashboard_get_validation_sets):
    fake_dashboard_get_validation_sets.return_value = validation_sets.ValidationSets.unmarshal(
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
                {
                    "headers": {
                        "account-id": "AccountIDXXXOfTheRequestingUserX",
                        "authority-id": "AccountIDXXXOfTheRequestingUserX",
                        "name": "acme-qa",
                        "revision": "2",
                        "sequence": "2",
                        "series": "16",
                        "sign-key-sha3-384": "XSignXKeyXHashXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
                        "snaps": [
                            {
                                "id": "XXSnapIDForXSnapName3XXXXXXXXXXX",
                                "name": "snap-name-3",
                                "presence": "required",
                                "revision": "2",
                            }
                        ],
                        "timestamp": "2020-10-29T16:36:56Z",
                        "type": "validation-set",
                    }
                },
            ]
        }
    )

    cmd = ["list-validation-sets"]

    result = click_run(cmd)

    assert result.exit_code == 0
    assert result.output == dedent(
        """\
        Account-ID                        Name              Sequence    Revision    When
        AccountIDXXXOfTheRequestingUserX  certification-x1  9           222         2020-10-29
        AccountIDXXXOfTheRequestingUserX  acme-qa           2           2           2020-10-29
        """
    )
    fake_dashboard_get_validation_sets.assert_called_once_with(
        name=None, sequence=None,
    )
