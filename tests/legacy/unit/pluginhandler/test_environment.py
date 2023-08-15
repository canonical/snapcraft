# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2023 Canonical Ltd
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

import sys

import pytest

from snapcraft_legacy.internal.pluginhandler._part_environment import (
    get_snapcraft_global_environment,
)
from snapcraft_legacy.project import Project
from tests.legacy.fixture_setup import SnapcraftYaml


@pytest.fixture(autouse=True)
def mock_platform(mocker):
    mocker.patch.object(sys, "platform", "linux")
    mocker.patch("platform.machine", return_value="aarch64")


def test_no_build_for_arch(tmp_path):
    """build-for envvars should exist when the build-for arch can be determined."""
    snapcraft_yaml = SnapcraftYaml(
        tmp_path,
        base="core20",
        parts={"test-part": {"plugin": "nil"}},
        architectures=[{"build-on": ["amd64", "arm64"], "run-on": "amd64"}],
    )
    snapcraft_yaml.write_snapcraft_yaml()
    project = Project(snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)

    environment = get_snapcraft_global_environment(project)

    assert environment["SNAPCRAFT_ARCH_BUILD_ON"] == project.arch_build_on
    assert (
        environment["SNAPCRAFT_ARCH_TRIPLET_BUILD_ON"] == project.arch_triplet_build_on
    )
    assert environment["SNAPCRAFT_ARCH_BUILD_FOR"] == project.arch_build_for
    assert (
        environment["SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR"]
        == project.arch_triplet_build_for
    )


def test_implicit_build_for_arch(tmp_path):
    """build-for envvars should exist for when the build-for arch is implicit."""
    snapcraft_yaml = SnapcraftYaml(
        tmp_path,
        base="core20",
        parts={"test-part": {"plugin": "nil"}},
        architectures=[{"build-on": "arm64"}],
    )
    snapcraft_yaml.write_snapcraft_yaml()
    project = Project(snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)

    environment = get_snapcraft_global_environment(project)

    assert environment["SNAPCRAFT_ARCH_BUILD_ON"] == project.arch_build_on
    assert (
        environment["SNAPCRAFT_ARCH_TRIPLET_BUILD_ON"] == project.arch_triplet_build_on
    )
    assert environment["SNAPCRAFT_ARCH_BUILD_FOR"] == project.arch_build_for
    assert (
        environment["SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR"]
        == project.arch_triplet_build_for
    )


def test_no_build_for_unknown_arch(tmp_path):
    """build-for envvars should not be defined for unknown build-for architectures."""
    snapcraft_yaml = SnapcraftYaml(
        tmp_path,
        base="core20",
        parts={"test-part": {"plugin": "nil"}},
        architectures=[{"build-on": ["amd64", "arm64"], "run-on": "unknown-arch"}],
    )
    snapcraft_yaml.write_snapcraft_yaml()
    project = Project(snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)

    environment = get_snapcraft_global_environment(project)
    assert environment["SNAPCRAFT_ARCH_BUILD_ON"] == project.arch_build_on
    assert (
        environment["SNAPCRAFT_ARCH_TRIPLET_BUILD_ON"] == project.arch_triplet_build_on
    )
    assert "SNAPCRAFT_ARCH_BUILD_FOR" not in environment
    assert "SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR" not in environment


def test_no_build_for_multi_arch(tmp_path):
    """build-for envvars should not be defined for multi-arch builds."""
    snapcraft_yaml = SnapcraftYaml(
        tmp_path,
        base="core20",
        parts={"test-part": {"plugin": "nil"}},
        architectures=[{"build-on": ["amd64", "arm64"], "run-on": ["amd64", "arm64"]}],
    )
    snapcraft_yaml.write_snapcraft_yaml()
    project = Project(snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)

    environment = get_snapcraft_global_environment(project)

    assert environment["SNAPCRAFT_ARCH_BUILD_ON"] == project.arch_build_on
    assert (
        environment["SNAPCRAFT_ARCH_TRIPLET_BUILD_ON"] == project.arch_triplet_build_on
    )
    assert "SNAPCRAFT_ARCH_BUILD_FOR" not in environment
    assert "SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR" not in environment
