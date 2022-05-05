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

import copy

import pydantic
import pytest

from snapcraft.parts.validation import validate_part


def test_part_validation_data_type():
    with pytest.raises(TypeError) as raised:
        validate_part("invalid data")  # type: ignore

    assert str(raised.value) == "value must be a dictionary"


def test_part_validation_immutable():
    data = {
        "plugin": "make",
        "source": "foo",
        "make-parameters": ["-C bar"],
    }
    data_copy = copy.deepcopy(data)

    validate_part(data)

    assert data == data_copy


def test_part_validation_extra():
    data = {
        "plugin": "make",
        "source": "foo",
        "make-parameters": ["-C bar"],
        "unexpected-extra": True,
    }

    error = r"unexpected-extra\s+extra fields not permitted"
    with pytest.raises(pydantic.ValidationError, match=error):
        validate_part(data)


def test_part_validation_missing():
    data = {
        "plugin": "make",
        "make-parameters": ["-C bar"],
    }

    error = r"source\s+field required"
    with pytest.raises(pydantic.ValidationError, match=error):
        validate_part(data)
