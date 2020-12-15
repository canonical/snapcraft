# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
from textwrap import dedent

import pytest

from snapcraft.internal import project_loader
from snapcraft.project import Project


def get_project_config(snapcraft_yaml_content):
    snapcraft_yaml_path = pathlib.Path("snapcraft.yaml")
    with snapcraft_yaml_path.open("w") as snapcraft_yaml_file:
        print(snapcraft_yaml_content, file=snapcraft_yaml_file)

    project = Project(snapcraft_yaml_file_path=snapcraft_yaml_path.as_posix())
    return project_loader.load_config(project)


def test_build_packages_from_snapcraft_yaml(tmp_work_path):
    snapcraft_yaml = dedent(
        """\
        name: test
        base: core18
        version: "1.0"
        summary: test
        description: test
        confinement: strict
        grade: stable

        build-packages: [foobar]

        parts:
          part1:
            plugin: nil
            build-packages: [foo, bar]
        """
    )

    project_config = get_project_config(snapcraft_yaml)

    assert project_config.get_build_packages() == {"foobar", "foo", "bar"}


@pytest.mark.parametrize(
    "source,expected_package",
    [("git://github.com/ubuntu-core/snapcraft.git", "git"), ("lp:ubuntu-push", "bzr")],
)
def test_config_adds_vcs_packages_to_build_packages(
    tmp_work_path, source, expected_package
):
    snapcraft_yaml = dedent(
        f"""\
            name: test
            base: core18
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                source: {source}
                plugin: nil
            """
    )

    project_config = get_project_config(snapcraft_yaml)

    assert expected_package in project_config.get_build_packages()


@pytest.mark.parametrize(
    "source_type,expected_package",
    [
        ("git", "git"),
        ("hg", "mercurial"),
        ("mercurial", "mercurial"),
        ("bzr", "bzr"),
        ("svn", "subversion"),
        ("subversion", "subversion"),
    ],
)
def test_config_adds_vcs_packages_to_build_packages_from_types(
    tmp_work_path, source_type, expected_package
):
    snapcraft_yaml = dedent(
        f"""\
            name: test
            base: core18
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                source: http://something/somewhere
                source-type: {source_type}
                plugin: autotools
        """
    )

    project_config = get_project_config(snapcraft_yaml)

    assert expected_package in project_config.get_build_packages()


def test_git_added_for_version_git(tmp_work_path):
    snapcraft_yaml = dedent(
        """\
            name: test
            base: core18
            version: "git"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
            """
    )

    project_config = get_project_config(snapcraft_yaml)

    "git" in project_config.get_build_packages()
