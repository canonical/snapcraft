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

from snapcraft import utils


@pytest.mark.parametrize(
    "value",
    [
        "y",
        "Y",
        "yes",
        "YES",
        "Yes",
        "t",
        "T",
        "true",
        "TRUE",
        "True",
        "On",
        "ON",
        "oN",
        "1",
    ],
)
def test_strtobool_true(value: str):
    assert utils.strtobool(value) is True


@pytest.mark.parametrize(
    "value",
    [
        "n",
        "N",
        "no",
        "NO",
        "No",
        "f",
        "F",
        "false",
        "FALSE",
        "False",
        "off",
        "OFF",
        "oFF",
        "0",
    ],
)
def test_strtobool_false(value: str):
    assert utils.strtobool(value) is False


@pytest.mark.parametrize(
    "value",
    [
        "not",
        "yup",
        "negative",
        "positive",
        "whatever",
        "2",
        "3",
    ],
)
def test_strtobool_value_error(value: str):
    with pytest.raises(ValueError):
        utils.strtobool(value)
