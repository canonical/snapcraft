# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021,2024 Canonical Ltd
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

from snapcraft_legacy.storeapi.v2 import validation_sets


@pytest.fixture()
def fake_snap_data():
    return {
        "name": "snap-name",
        "id": "snap-id",
        "presence": "required",
        "revision": 42,
    }


@pytest.fixture()
def fake_snap(fake_snap_data):
    return validation_sets.Snap.unmarshal(fake_snap_data)


@pytest.fixture()
def fake_editable_build_assertion_data(fake_snap_data):
    return {
        "account-id": "account-id-1",
        "name": "validation-set-1",
        "revision": "revision-1",
        "sequence": 1,
        "snaps": [fake_snap_data],
    }


@pytest.fixture()
def fake_editable_build_assertion(fake_editable_build_assertion_data):
    return validation_sets.EditableBuildAssertion.unmarshal(
        fake_editable_build_assertion_data
    )


@pytest.fixture()
def fake_build_assertion_data(fake_snap_data):
    return {
        "account-id": "account-id-1",
        "name": "validation-set-1",
        "revision": "revision-1",
        "sequence": 1,
        "snaps": [fake_snap_data],
        "authority-id": "authority-id-1",
        "series": "16",
        "sign-key-sha3-384": "sign-key-1",
        "timestamp": "2020-10-29T16:36:56Z",
        "type": "validation-set",
    }


@pytest.fixture()
def fake_build_assertion(fake_build_assertion_data):
    return validation_sets.BuildAssertion.unmarshal(fake_build_assertion_data)


def test_ignore_sign_key(fake_build_assertion):
    """Ignore the sign key when unmarshalling."""
    assert not fake_build_assertion.sign_key_sha3_384


def test_editable_build_assertion_marshal_as_str(fake_editable_build_assertion):
    """Cast all scalars to string when marshalling."""
    data = fake_editable_build_assertion.marshal_as_str()

    assert data == {
        "account-id": "account-id-1",
        "name": "validation-set-1",
        "revision": "revision-1",
        "sequence": "1",
        "snaps": [
            {
                "id": "snap-id",
                "name": "snap-name",
                "presence": "required",
                "revision": "42",
            },
        ],
    }


def test_build_assertion_marshal_as_str(fake_build_assertion):
    """Cast all scalars to string when marshalling."""
    data = fake_build_assertion.marshal_as_str()

    assert data == {
        "account-id": "account-id-1",
        "authority-id": "authority-id-1",
        "name": "validation-set-1",
        "revision": "revision-1",
        "sequence": "1",
        "series": "16",
        "snaps": [
            {
                "id": "snap-id",
                "name": "snap-name",
                "presence": "required",
                "revision": "42",
            },
        ],
        "timestamp": "2020-10-29T16:36:56Z",
        "type": "validation-set",
    }
