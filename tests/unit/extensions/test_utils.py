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


import pytest

from snapcraft.extensions._utils import _apply_extension_property


@pytest.mark.parametrize(
    "existing_property,extension_property,expected_value",
    [
        # prepend
        (
            ["item3", "item4", "item5"],
            ["item1", "item2"],
            ["item1", "item2", "item3", "item4", "item5"],
        ),
        # empty extension
        (["item3", "item4", "item5"], [], ["item3", "item4", "item5"]),
        # empty property
        ([], ["item1", "item2"], ["item1", "item2"]),
        # duplicate items keeps first found
        (
            ["item3", "item4", "item1"],
            ["item1", "item2"],
            ["item1", "item2", "item3", "item4"],
        ),
        # non scalar
        (
            [{"k2": "v2"}],
            [{"k1": "v1"}],
            [{"k1": "v1"}, {"k2": "v2"}],
        ),
    ],
)
def test_apply_property_list(existing_property, extension_property, expected_value):
    assert (
        _apply_extension_property(existing_property, extension_property)
        == expected_value
    )


@pytest.mark.parametrize(
    "existing_property,extension_property,expected_value",
    [
        # add
        (
            {"k1": "v1", "k2": "v2", "k3": "v3"},
            {"k4": "v4", "k5": "v5"},
            {"k1": "v1", "k2": "v2", "k3": "v3", "k4": "v4", "k5": "v5"},
        ),
        # conflicts keeps existing property
        (
            {"k1": "v1", "k2": "v2", "k3": "v3"},
            {"k3": "nv3", "k4": "v4"},
            {"k1": "v1", "k2": "v2", "k3": "v3", "k4": "v4"},
        ),
        # empty property
        ({}, {"k4": "v4", "k5": "v5"}, {"k4": "v4", "k5": "v5"}),
    ],
)
def test_apply_property_dictionary(
    existing_property, extension_property, expected_value
):
    assert (
        _apply_extension_property(existing_property, extension_property)
        == expected_value
    )
