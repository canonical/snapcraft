# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019, 2023 Canonical Ltd
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
import sys
from datetime import datetime
from textwrap import dedent

import pytest

from snapcraft_legacy.project import Project
from tests.legacy.fixture_setup import SnapcraftYaml


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


@pytest.mark.parametrize(
    ["architectures", "expected_run_on", "expected_run_on_triplet"],
    [
        # implicitly defined run-on arch
        (None, "arm64", "aarch64-linux-gnu"),
        (["arm64"], "arm64", "aarch64-linux-gnu"),
        ([{"build-on": "arm64"}], "arm64", "aarch64-linux-gnu"),
        ([{"build-on": ["arm64"]}], "arm64", "aarch64-linux-gnu"),
        # explicitly defined run-on arch
        ([{"build-on": "arm64", "run-on": "amd64"}], "amd64", "x86_64-linux-gnu"),
        ([{"build-on": ["arm64"], "run-on": ["amd64"]}], "amd64", "x86_64-linux-gnu"),
        (
            [
                {"build-on": "amd64", "run-on": "amd64"},
                {"build-on": "arm64", "run-on": "amd64"},
            ],
            "amd64",
            "x86_64-linux-gnu",
        ),
        (
            [
                {"build-on": ["amd64"], "run-on": ["amd64"]},
                {"build-on": ["arm64"], "run-on": ["amd64"]},
            ],
            "amd64",
            "x86_64-linux-gnu",
        ),
        # multi-arch builds - run-on arch cannot be determined
        (["arm64", "amd64"], None, None),
        ([{"build-on": ["arm64", "amd64"]}], None, None),
        ([{"build-on": "arm64", "run-on": ["arm64", "amd64"]}], None, None),
        ([{"build-on": ["arm64"], "run-on": ["arm64", "amd64"]}], None, None),
        # unknown run-on arch - run-on arch cannot be determined
        ([{"build-on": "arm64", "run-on": "bad-arch"}], None, None),
        ([{"build-on": ["arm64"], "run-on": ["bad-arch"]}], None, None),
    ],
)
def test_project_architectures(
    architectures, expected_run_on, expected_run_on_triplet, mocker, tmp_path
):
    """Verify build-on and run-on architectures are correctly determined."""
    mocker.patch.object(sys, "platform", "linux")
    mocker.patch("platform.machine", return_value="aarch64")
    snapcraft_yaml = SnapcraftYaml(
        tmp_path,
        base="core20",
        parts={"test-part": {"plugin": "nil"}},
        architectures=architectures,
    )
    snapcraft_yaml.write_snapcraft_yaml()

    project = Project(snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)

    assert project.arch_build_on == "arm64"
    assert project.arch_triplet_build_on == "aarch64-linux-gnu"
    assert project.arch_run_on == expected_run_on
    assert project.arch_triplet_run_on == expected_run_on_triplet


@pytest.mark.parametrize(
    "architectures",
    [
        # empty data
        [],
        [{}],
        [{"build-on": None}],
        [{"build-on": None, "run-on": None}],
        # not a list
        "",
        "arm64",
        {"build-on": {"build-on": "arm64"}},
        # wrong data type inside the list
        ["arm64", {"build-on": "arm64"}],
        [{"build-on": None}],
        [{"build-on": {"build-on": "arm64"}}],
        # missing build-on or run-on
        [{"foo": "bar"}],
        [{"build-on": "arm64", "foo": "bar"}],
        [{"run-on": "arm64"}],
        [{"run-on": "arm64", "foo": "bar"}],
        # valid structure but invalid contents
        ["arm64", "arm64"],
        [{"build-on": "arm64"}, {"build-on": "arm64"}],
        [
            {"build-on": "arm64", "run-on": "arm64"},
            {"build-on": "arm64", "run-on": "arm64"},
        ],
    ],
)
def test_project_architectures_bad_data(architectures, mocker, tmp_path):
    """Architecture data is validated later in the lifecycle with helpful error
    messages, so ensure bad data does not raise an error here.
    """
    mocker.patch.object(sys, "platform", "linux")
    mocker.patch("platform.machine", return_value="aarch64")
    snapcraft_yaml = SnapcraftYaml(
        tmp_path,
        base="core20",
        parts={"test-part": {"plugin": "nil"}},
        architectures=architectures,
    )
    snapcraft_yaml.write_snapcraft_yaml()

    Project(snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path)


@pytest.mark.parametrize(
    "architectures",
    [
        None,
        "arm64",
        [{"build-on": "arm64", "run-on": "amd64"}],
    ],
)
def test_project_architectures_target_arch(architectures, mocker, tmp_path):
    """When `target_deb_arch` is provided, it is always used for the run-on arch."""
    mocker.patch.object(sys, "platform", "linux")
    mocker.patch("platform.machine", return_value="aarch64")
    snapcraft_yaml = SnapcraftYaml(
        tmp_path,
        base="core20",
        parts={"test-part": {"plugin": "nil"}},
        architectures=architectures,
    )
    snapcraft_yaml.write_snapcraft_yaml()

    project = Project(
        snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path,
        # choose a target arch not defined in the snapcraft.yaml
        target_deb_arch="riscv64",
    )

    assert project.arch_build_on == "arm64"
    assert project.arch_triplet_build_on == "aarch64-linux-gnu"
    assert project.arch_run_on == "riscv64"
    assert project.arch_triplet_run_on == "riscv64-linux-gnu"
