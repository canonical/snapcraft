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


import io
from textwrap import dedent

import pytest

from snapcraft import errors
from snapcraft.parts import yaml_utils


def test_yaml_load():
    assert (
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
        entry:
            sub-entry:
              - list1
              - list2
        scalar: scalar-value
    """
                )
            )
        )
        == {
            "entry": {
                "sub-entry": ["list1", "list2"],
            },
            "scalar": "scalar-value",
        }
    )


def test_yaml_load_duplicates_errors():
    with pytest.raises(errors.SnapcraftError) as raised:
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
            entry: value1
            entry: value2
    """
                )
            )
        )

    assert str(raised.value) == dedent(
        """\
        YAML parsing error: while constructing a mapping
        found duplicate key 'entry'
          in "<file>", line 1, column 1"""
    )


def test_yaml_load_unhashable_errors():
    with pytest.raises(errors.SnapcraftError) as raised:
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
            entry: {{value}}
    """
                )
            )
        )

    assert str(raised.value) == dedent(
        """\
        YAML parsing error: while constructing a mapping
        found unhashable key
          in "<file>", line 1, column 8"""
    )
