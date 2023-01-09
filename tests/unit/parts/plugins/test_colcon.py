# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import os
import sys
from pathlib import Path

import pytest
from craft_parts import Part, PartInfo, ProjectInfo
from pydantic import ValidationError

import snapcraft.parts.plugins as colcon
from snapcraft.parts.plugins import _ros


def test_rosdep_error():
    e = _ros.RosdepUnexpectedResultError(dependency="foo", output="foo doesn't exists")
    assert (
        str(e) == "Received unexpected result from rosdep when "
        "trying to resolve 'foo':\nfoo doesn't exists"
    )


def test_parse_rosdep_resolve_dependencies():
    # @todo
    assert True


@pytest.fixture
def setup_method_fixture():
    def _setup_method_fixture(new_dir, properties=None):
        if properties is None:
            properties = {}
        properties["source"] = "."
        plugin_properties = colcon.ColconPlugin.properties_class.unmarshal(properties)
        part = Part("foo", {})

        project_info = ProjectInfo(
            application_name="test", base="core22", cache_dir=new_dir
        )
        project_info._parallel_build_count = 42

        part_info = PartInfo(project_info=project_info, part=part)
        part_info._part_install_dir = Path("install/dir")

        return colcon.ColconPlugin(properties=plugin_properties, part_info=part_info)

    yield _setup_method_fixture


class TestPluginColconPlugin:
    """Colcon plugin tests."""

    def test_property_empty_invalid(self):
        with pytest.raises(ValidationError):
            colcon.ColconPlugin.properties_class.unmarshal({})

    def test_property_default(self):
        try:
            colcon.ColconPlugin.properties_class.unmarshal({"source": "."})
        except ValidationError as e:
            raise AssertionError(f"{e}")

    def test_property_all(self):
        try:
            properties = colcon.ColconPlugin.properties_class.unmarshal(
                {
                    "source": ".",
                    "colcon-ament-cmake-args": ["ament", "args..."],
                    "colcon-catkin-cmake-args": ["catkin", "args..."],
                    "colcon-cmake-args": ["cmake", "args..."],
                    "colcon-packages": ["package1", "package2..."],
                    "colcon-packages-ignore": ["ipackage1", "ipackage2..."],
                }
            )
        except ValidationError as e:
            raise AssertionError(f"{e}")

        assert properties.source == "."  # type: ignore
        assert properties.colcon_ament_cmake_args == ["ament", "args..."]  # type: ignore
        assert properties.colcon_catkin_cmake_args == ["catkin", "args..."]  # type: ignore
        assert properties.colcon_cmake_args == ["cmake", "args..."]  # type: ignore
        assert properties.colcon_packages == ["package1", "package2..."]  # type: ignore
        assert properties.colcon_packages_ignore == ["ipackage1", "ipackage2..."]  # type: ignore

    def test_get_build_packages(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

        assert plugin.get_build_packages() == {
            "python3-colcon-common-extensions",
            "python3-rosdep",
            "python3-rosinstall",
            "python3-wstool",
        }

    def test_get_build_environment(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

        assert plugin.get_build_environment() == {
            "AMENT_PYTHON_EXECUTABLE": "/usr/bin/python3",
            "COLCON_PYTHON_EXECUTABLE": "/usr/bin/python3",
            "ROS_PYTHON_VERSION": "3",
        }

    def test_out_of_source_build_property(self):
        assert colcon.ColconPlugin.get_out_of_source_build

    def test_get_build_commands(self, setup_method_fixture, new_dir, monkeypatch):
        plugin = setup_method_fixture(new_dir)

        monkeypatch.setattr(sys, "path", ["", "/test"])
        monkeypatch.setattr(sys, "executable", "/test/python3")
        monkeypatch.setattr(_ros, "__file__", "/test/_ros.py")
        monkeypatch.setattr(os, "environ", {})

        assert plugin.get_build_commands() == [
            'state="$(set +o); set -$-"',
            "set +u",
            'if [ -f "${CRAFT_PART_INSTALL}/opt/ros/snap/setup.sh" ]; then',
            'COLCON_CURRENT_PREFIX="${CRAFT_PART_INSTALL}/opt/ros/snap" . '
            '"${CRAFT_PART_INSTALL}/opt/ros/snap/setup.sh"',
            "fi",
            '. "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
            'eval "${state}"',
            "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep "
            "init; fi",
            'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
            "rosdep install --default-yes --ignore-packages-from-source "
            '--from-paths "${CRAFT_PART_SRC_WORK}"',
            "colcon build "
            '--base-paths "${CRAFT_PART_SRC_WORK}" --build-base "${CRAFT_PART_BUILD}" '
            '--merge-install --install-base "${CRAFT_PART_INSTALL}/opt/ros/snap" '
            '--parallel-workers "${CRAFT_PARALLEL_BUILD_COUNT}"',
            'if [ -f "${CRAFT_PART_INSTALL}/opt/ros/snap/COLCON_IGNORE" ]; then',
            'rm "${CRAFT_PART_INSTALL}/opt/ros/snap/COLCON_IGNORE"',
            "fi",
            "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 /test/python3 -I "
            "/test/_ros.py "
            'stage-runtime-dependencies --part-src "${CRAFT_PART_SRC_WORK}" '
            '--part-install "${CRAFT_PART_INSTALL}" '
            '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" '
            '--target-arch "${CRAFT_TARGET_ARCH}" '
            f"--stage-cache-dir {new_dir} --base core22",
        ]

    def test_get_build_commands_with_all_properties(
        self, setup_method_fixture, new_dir, monkeypatch
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "source": ".",
                "colcon-ament-cmake-args": ["ament", "args..."],
                "colcon-catkin-cmake-args": ["catkin", "args..."],
                "colcon-cmake-args": ["cmake", "args..."],
                "colcon-packages": ["package1", "package2..."],
                "colcon-packages-ignore": ["ipackage1", "ipackage2..."],
            },
        )

        monkeypatch.setattr(sys, "path", ["", "/test"])
        monkeypatch.setattr(sys, "executable", "/test/python3")
        monkeypatch.setattr(_ros, "__file__", "/test/_ros.py")
        monkeypatch.setattr(
            os,
            "environ",
            dict(
                FOO="baR",
                PATH="/bin:/test",
                SNAP="TESTSNAP",
                SNAP_ARCH="TESTARCH",
                SNAP_NAME="TESTSNAPNAME",
                SNAP_VERSION="TESTV1",
                http_proxy="http://foo",
                https_proxy="https://bar",
            ),
        )

        assert plugin.get_build_commands() == [
            'state="$(set +o); set -$-"',
            "set +u",
            'if [ -f "${CRAFT_PART_INSTALL}/opt/ros/snap/setup.sh" ]; then',
            'COLCON_CURRENT_PREFIX="${CRAFT_PART_INSTALL}/opt/ros/snap" . '
            '"${CRAFT_PART_INSTALL}/opt/ros/snap/setup.sh"',
            "fi",
            '. "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
            'eval "${state}"',
            "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep "
            "init; fi",
            'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
            "rosdep install --default-yes --ignore-packages-from-source "
            '--from-paths "${CRAFT_PART_SRC_WORK}"',
            "colcon build "
            '--base-paths "${CRAFT_PART_SRC_WORK}" --build-base "${CRAFT_PART_BUILD}" '
            '--merge-install --install-base "${CRAFT_PART_INSTALL}/opt/ros/snap" '
            "--packages-ignore ipackage1 ipackage2... --packages-select package1 "
            "package2... --cmake-args cmake args... "
            "--ament-cmake-args ament args... --catkin-cmake-args catkin "
            'args... --parallel-workers "${CRAFT_PARALLEL_BUILD_COUNT}"',
            'if [ -f "${CRAFT_PART_INSTALL}/opt/ros/snap/COLCON_IGNORE" ]; then',
            'rm "${CRAFT_PART_INSTALL}/opt/ros/snap/COLCON_IGNORE"',
            "fi",
            "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 PATH=/bin:/test SNAP=TESTSNAP "
            "SNAP_ARCH=TESTARCH SNAP_NAME=TESTSNAPNAME SNAP_VERSION=TESTV1 "
            "http_proxy=http://foo https_proxy=https://bar "
            "/test/python3 -I /test/_ros.py "
            'stage-runtime-dependencies --part-src "${CRAFT_PART_SRC_WORK}" '
            '--part-install "${CRAFT_PART_INSTALL}" '
            '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" '
            '--target-arch "${CRAFT_TARGET_ARCH}" '
            f"--stage-cache-dir {new_dir} --base core22",
        ]
