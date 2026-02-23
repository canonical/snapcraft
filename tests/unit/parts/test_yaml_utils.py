# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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
import pathlib
import re
from textwrap import dedent

import craft_application.errors
import pytest
from craft_parts import ProjectVar, ProjectVarInfo

from snapcraft import const, errors
from snapcraft.parts import yaml_utils


def test_yaml_load():
    assert yaml_utils.load(
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
    ) == {
        "base": "core22",
        "entry": {
            "sub-entry": ["list1", "list2"],
        },
        "scalar": "scalar-value",
    }


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
    assert yaml_utils.load(
        io.StringIO(
            dedent(
                """\
        base: foo
        build-base: core22
    """
            )
        )
    ) == {
        "base": "foo",
        "build-base": "core22",
    }


@pytest.mark.parametrize("base", const.ESM_BASES)
def test_yaml_load_esm_base(base):
    with pytest.raises(errors.MaintenanceBase):
        yaml_utils.load(
            io.StringIO(
                dedent(
                    f"""\
                    base: {base}
                    """
                )
            )
        )


def test_yaml_load_no_base():
    expected = re.escape("Project file has no base or build-base.")

    with pytest.raises(errors.MissingBase, match=expected):
        yaml_utils.load(
            io.StringIO(
                dedent(
                    """\
                    entry: foo
                    """
                )
            )
        )


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
        "architectures": [{"build-on": "amd64", "build-for": "amd64"}],
        "parts": {"nil": {}, "snapcraft/core": {"plugin": "nil", key: ["foo"]}},
    }


def test_get_base(mocker):
    mock_get_effective_base = mocker.patch(
        "snapcraft.parts.yaml_utils.utils.get_effective_base",
        return_value="test-effective-base",
    )
    yaml = io.StringIO(
        dedent(
            """\
            name: test-name
            type: test-type
            base: test-base
            build-base: test-build-base
            """
        )
    )

    effective_base = yaml_utils.get_base(yaml)

    mock_get_effective_base.assert_called_with(
        base="test-base",
        build_base="test-build-base",
        name="test-name",
        project_type="test-type",
    )
    assert effective_base == "test-effective-base"


def test_get_base_from_yaml(mocker):
    mock_get_effective_base = mocker.patch(
        "snapcraft.parts.yaml_utils.utils.get_effective_base",
        return_value="test-effective-base",
    )
    yaml_dict = {
        "name": "test-name",
        "type": "test-type",
        "base": "test-base",
        "build-base": "test-build-base",
    }

    effective_base = yaml_utils.get_base_from_yaml(yaml_dict)

    mock_get_effective_base.assert_called_with(
        base="test-base",
        build_base="test-build-base",
        name="test-name",
        project_type="test-type",
    )
    assert effective_base == "test-effective-base"


@pytest.mark.parametrize("project", yaml_utils._SNAP_PROJECT_FILES)
@pytest.mark.parametrize("project_dir", [None, "test-project-dir"])
def test_get_snap_project(project, project_dir, new_dir):
    project_dir = pathlib.Path(project_dir) if project_dir else new_dir
    (project_dir / project.project_file).parent.mkdir(parents=True, exist_ok=True)
    (project_dir / project.project_file).touch()

    actual_project = yaml_utils.get_snap_project(project_dir)

    assert actual_project == project


@pytest.mark.parametrize("project_dir", [None, "test-project-dir"])
def test_get_snap_project_snap_not_a_directory(project_dir, new_dir):
    project_dir = pathlib.Path(project_dir) if project_dir else new_dir
    project_dir.mkdir(parents=True, exist_ok=True)
    (project_dir / "snap").touch()

    with pytest.raises(craft_application.errors.ProjectDirectoryTypeError) as raised:
        yaml_utils.get_snap_project(project_dir)

    assert "Given project directory path is not a directory:" in str(raised.value)


@pytest.mark.parametrize(
    ("data", "expected"),
    [
        pytest.param(
            {},
            ProjectVarInfo.unmarshal(
                {
                    "version": ProjectVar().marshal(),
                    "grade": ProjectVar().marshal(),
                }
            ),
            id="minimal",
        ),
        pytest.param(
            {
                "version": "1.0",
                "grade": "stable",
                "adopt-info": "part1",
                "components": {
                    "my-component": {
                        "version": "2.0",
                        "adopt-info": "part2",
                    },
                    "your-component": {
                        "version": "3.0",
                        "adopt-info": "part3",
                    },
                },
            },
            ProjectVarInfo.unmarshal(
                {
                    "version": ProjectVar(
                        value="1.0",
                        part_name="part1",
                    ).marshal(),
                    "grade": ProjectVar(
                        value="stable",
                        part_name="part1",
                    ).marshal(),
                    "components": {
                        "my-component": {
                            "version": ProjectVar(
                                value="2.0",
                                part_name="part2",
                            ).marshal(),
                        },
                        "your-component": {
                            "version": ProjectVar(
                                value="3.0",
                                part_name="part3",
                            ).marshal(),
                        },
                    },
                }
            ),
            id="complex",
        ),
    ],
)
def test_create_project_vars(data, expected):
    """Create a minimal project var."""
    project_vars = yaml_utils.create_project_vars(
        {
            "name": "name",
            "base": "core22",
            "confinement": "strict",
            "summary": "summary",
            "description": "description",
            "parts": {"nil": {}},
            **data,
        }
    )

    assert project_vars == expected
