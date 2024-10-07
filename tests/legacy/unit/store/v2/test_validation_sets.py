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
        "components": {
            "component-with-revision": {
                "presence": "required",
                "revision": 10,
            },
            "component-without-revision": "invalid",
        },
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


@pytest.mark.parametrize(
    ("input_dict", "expected"),
    [
        pytest.param({}, {}, id="empty"),
        pytest.param(
            {False: False, True: True},
            {"False": "False", "True": "True"},
            id="boolean values",
        ),
        pytest.param(
            {0: 0, None: None, "dict": {}, "list": [], "str": ""},
            ({"0": "0", None: None, "dict": {}, "list": [], "str": ""}),
            id="none-like values",
        ),
        pytest.param(
            {10: 10, 20.0: 20.0, "30": "30", True: True},
            {"10": "10", "20.0": "20.0", "30": "30", "True": "True"},
            id="scalar values",
        ),
        pytest.param(
            {"foo": {"bar": [1, 2.0], "baz": {"qux": True}}},
            {"foo": {"bar": ["1", "2.0"], "baz": {"qux": "True"}}},
            id="nested data structures",
        ),
    ],
)
def test_cast_dict_scalars_to_strings(input_dict, expected):
    actual = validation_sets.cast_dict_scalars_to_strings(input_dict)

    assert actual == expected


def test_ignore_sign_key(fake_build_assertion):
    """Ignore the sign key when unmarshalling."""
    assert not fake_build_assertion.sign_key_sha3_384


def test_editable_build_assertion_marshal_as_str(fake_editable_build_assertion):
    """Cast all scalars to string when marshalling."""
    data = fake_editable_build_assertion.marshal_scalars_as_strings()

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
                "components": {
                    "component-with-revision": {
                        "presence": "required",
                        "revision": "10",
                    },
                    "component-without-revision": "invalid",
                },
            },
        ],
    }


def test_build_assertion_marshal_as_str(fake_build_assertion):
    """Cast all scalars to string when marshalling."""
    data = fake_build_assertion.marshal_scalars_as_strings()

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
                "components": {
                    "component-with-revision": {
                        "presence": "required",
                        "revision": "10",
                    },
                    "component-without-revision": "invalid",
                },
            },
        ],
        "timestamp": "2020-10-29T16:36:56Z",
        "type": "validation-set",
    }
