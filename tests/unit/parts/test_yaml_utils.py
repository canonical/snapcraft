# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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
from snapcraft.projects import Architecture


def test_yaml_load():
    assert (
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
        base: core22
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
            "base": "core22",
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
            base: core22
            entry: value1
            entry: value2
    """
                )
            )
        )

    assert str(raised.value) == dedent(
        """\
        snapcraft.yaml parsing error: while constructing a mapping
        found duplicate key 'entry'
          in "<file>", line 1, column 1"""
    )


def test_yaml_load_unhashable_errors():
    with pytest.raises(errors.SnapcraftError) as raised:
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
            base: core22
            entry: {{value}}
    """
                )
            )
        )

    assert str(raised.value) == dedent(
        """\
        snapcraft.yaml parsing error: while constructing a mapping
          in "<file>", line 2, column 8
        found unhashable key
          in "<file>", line 2, column 9"""
    )


def test_yaml_load_build_base():
    assert (
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
        base: foo
        build-base: core22
    """
                )
            )
        )
        == {
            "base": "foo",
            "build-base": "core22",
        }
    )


def test_yaml_load_not_core22_base():
    with pytest.raises(errors.LegacyFallback) as raised:
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
            base: core20
    """
                )
            )
        )

    assert str(raised.value) == "base is core20"


def test_yaml_load_esm_base():
    with pytest.raises(errors.MaintenanceBase):
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
            base: core
    """
                )
            )
        )


def test_yaml_load_no_base():
    with pytest.raises(errors.LegacyFallback) as raised:
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
            entry: foo
    """
                )
            )
        )

    assert str(raised.value) == "no base defined"


def test_extract_parse_info():
    yaml_data = {
        "name": "foo",
        "parts": {"p1": {"plugin": "nil", "parse-info": "foo/metadata.xml"}, "p2": {}},
    }
    parse_info = yaml_utils.extract_parse_info(yaml_data)
    assert yaml_data == {"name": "foo", "parts": {"p1": {"plugin": "nil"}, "p2": {}}}
    assert parse_info == {"p1": "foo/metadata.xml"}


@pytest.fixture
def minimal_yaml_data():
    return {
        "name": "name",
        "base": "core22",
        "confinement": "strict",
        "grade": "devel",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "parts": {"nil": {}},
    }


@pytest.mark.parametrize("key", ("build-packages", "build-snaps"))
@pytest.mark.parametrize("value", (["foo"], [{"on amd64": ["foo"]}]))
def test_apply_yaml_defines_root_packages(minimal_yaml_data, key, value):
    minimal_yaml_data[key] = value

    assert yaml_utils.apply_yaml(
        minimal_yaml_data, build_on="amd64", build_for="amd64"
    ) == {
        "name": "name",
        "base": "core22",
        "confinement": "strict",
        "grade": "devel",
        "version": "1.0",
        "summary": "summary",
        "description": "description",
        "architectures": [Architecture(build_on="amd64", build_for="amd64")],
        "parts": {"nil": {}, "snapcraft/core": {"plugin": "nil", key: ["foo"]}},
    }
