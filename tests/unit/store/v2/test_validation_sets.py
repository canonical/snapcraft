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

import pytest

from snapcraft.storeapi.v2 import validation_sets


@pytest.mark.parametrize("snap_id", (None, "snap_id"))
@pytest.mark.parametrize("presence", (None, "required", "optional", "invalid"))
@pytest.mark.parametrize("revision", ("42", None))
def test_snap(snap_id, presence, revision):
    payload = {
        "name": "set-1",
    }

    if presence is not None:
        payload["presence"] = presence

    if snap_id is not None:
        payload["id"] = snap_id

    if revision is not None:
        payload["revision"] = revision

    s = validation_sets.Snap.unmarshal(payload)

    assert repr(s) == "<Snap: 'set-1'>"
    assert s.name == payload["name"]
    assert s.snap_id == payload.get("id")
    assert s.presence == payload.get("presence")
    assert s.revision == payload.get("revision")
    assert s.marshal() == payload


@pytest.mark.parametrize("revision", ("42", None))
def test_build_assertion(revision):
    payload = {
        "account-id": "account-id-1",
        "authority-id": "authority-id-1",
        "name": "validation-set-1",
        "sequence": "1",
        "series": "16",
        "snaps": [validation_sets.Snap(name="snap-name").marshal()],
        "timestamp": "2020-10-29T16:36:56Z",
        "type": "validation-set",
    }

    if revision is not None:
        payload["revision"] = revision

    s = validation_sets.BuildAssertion.unmarshal(payload)

    assert repr(s) == "<BuildAssertion: 'validation-set-1'>"
    assert s.account_id == payload["account-id"]
    assert s.authority_id == payload["authority-id"]
    assert s.name == payload["name"]
    assert s.sequence == payload.get("sequence")
    assert s.series == payload.get("series")
    assert s.snaps == [validation_sets.Snap(name="snap-name")]
    assert s.timestamp == "2020-10-29T16:36:56Z"

    if revision is None:
        payload["revision"] = "0"

    assert s.marshal() == payload


def test_validation_sets():
    payload = {
        "assertions": [
            {
                "headers": {
                    "account-id": "account-id-1",
                    "authority-id": "authority-id-1",
                    "name": "validation-set-1",
                    "revision": "1",
                    "sequence": "1",
                    "series": "16",
                    "snaps": [validation_sets.Snap(name="snap-name-1").marshal()],
                    "timestamp": "2020-10-29T16:36:56Z",
                    "type": "validation-set",
                }
            },
            {
                "headers": {
                    "account-id": "account-id-1",
                    "authority-id": "authority-id-1",
                    "name": "validation-set-1",
                    "revision": "3",
                    "sequence": "1",
                    "series": "16",
                    "snaps": [validation_sets.Snap(name="snap-name-2").marshal()],
                    "timestamp": "2020-10-29T16:36:56Z",
                    "type": "validation-set",
                },
            },
        ]
    }

    s = validation_sets.ValidationSets.unmarshal(payload)

    assert repr(s) == "<ValidationSets: assertions 2>"
    assert s.assertions[0] == validation_sets.BuildAssertion.unmarshal(
        {
            "account-id": "account-id-1",
            "authority-id": "authority-id-1",
            "name": "validation-set-1",
            "revision": "1",
            "sequence": "1",
            "series": "16",
            "snaps": [validation_sets.Snap(name="snap-name-1").marshal()],
            "timestamp": "2020-10-29T16:36:56Z",
            "type": "validation-set",
        },
    )

    assert s.assertions[1] == validation_sets.BuildAssertion.unmarshal(
        {
            "account-id": "account-id-1",
            "authority-id": "authority-id-1",
            "name": "validation-set-1",
            "revision": "3",
            "sequence": "1",
            "series": "16",
            "snaps": [validation_sets.Snap(name="snap-name-2").marshal()],
            "timestamp": "2020-10-29T16:36:56Z",
            "type": "validation-set",
        }
    )

    assert s.marshal() == payload
