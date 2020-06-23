# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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

import pathlib
from datetime import datetime
from textwrap import dedent

import pytest

from snapcraft.project import Project


def test_project_with_arguments():
    project = Project(target_deb_arch="armhf", debug=True)

    assert project.deb_arch == "armhf"
    assert project.debug is True

    # This is a backwards compatibility check
    assert project.info is None


def test_project_with_snapcraft_yaml_file_path_carries_info(tmp_work_path):
    snapcraft_yaml_path = pathlib.Path("snapcraft.yaml")
    with snapcraft_yaml_path.open("w") as snapcraft_yaml_file:
        print(
            dedent(
                """\
            name: foo
            version: "1"
            summary: bar
            description: baz
            confinement: strict

            parts:
              part1:
                plugin: go
            """
            ),
            file=snapcraft_yaml_file,
        )

    project = Project(snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix())

    # Only 1 value is enough
    assert project.info.name == "foo"


@pytest.mark.parametrize(
    "location", ["snap/snapcraft.yaml", "build-aux/snap/snapcraft.yaml"]
)
def test_project_local_plugin_location(tmp_work_path, location):
    snapcraft_yaml_path = tmp_work_path / pathlib.Path(location)
    snapcraft_yaml_path.parent.mkdir(parents=True)
    with snapcraft_yaml_path.open("w") as snapcraft_yaml_file:
        print(
            dedent(
                """\
            name: foo
            version: "1"
            summary: bar
            description: baz
            confinement: strict

            parts:
              part1:
                plugin: go
            """
            ),
            file=snapcraft_yaml_file,
        )

    project = Project(snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix())

    expected_plugins_dir = snapcraft_yaml_path.parent / "plugins"
    assert project.local_plugins_dir == expected_plugins_dir.as_posix()


def test_get_snapcraft_started():
    assert Project()._get_start_time() < datetime.utcnow()
