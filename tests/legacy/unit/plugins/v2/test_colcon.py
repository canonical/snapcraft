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

import snapcraft_legacy.plugins.v2._ros as _ros
import snapcraft_legacy.plugins.v2.colcon as colcon


def test_schema():
    assert colcon.ColconPlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "additionalProperties": False,
        "properties": {
            "colcon-ament-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
            },
            "colcon-catkin-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
            },
            "colcon-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
            },
            "colcon-packages": {
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
                "uniqueItems": True,
            },
            "colcon-packages-ignore": {
                "default": [],
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
                "uniqueItems": True,
            },
            "ros-build-snaps": {
                "type": "array",
                "minItems": 0,
                "uniqueItems": True,
                "items": {"type": "string"},
                "default": [],
            },
        },
        "type": "object",
    }


def test_get_build_packages():
    plugin = colcon.ColconPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_packages() == {
        "python3-colcon-common-extensions",
        "python3-rosdep",
        "python3-rosinstall",
        "python3-wstool",
        "rospack-tools",
    }


def test_get_build_snaps():
    class OptionsDefault:
        ros_build_snaps = list()

    plugin = colcon.ColconPlugin(part_name="my-part", options=OptionsDefault())

    assert plugin.get_build_snaps() == set()

    class Options:
        ros_build_snaps = ["Foo"]

    plugin = colcon.ColconPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_snaps() == {"Foo"}


def test_get_build_environment():
    plugin = colcon.ColconPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_environment() == {
        "AMENT_PYTHON_EXECUTABLE": "/usr/bin/python3",
        "COLCON_PYTHON_EXECUTABLE": "/usr/bin/python3",
        "ROS_PYTHON_VERSION": "3",
    }


def test_out_of_source_build_property():
    plugin = colcon.ColconPlugin(part_name="my-part", options=lambda: None)

    assert plugin.out_of_source_build


def test_get_build_commands(monkeypatch):
    class Options:
        colcon_ament_cmake_args = list()
        colcon_catkin_cmake_args = list()
        colcon_cmake_args = list()
        colcon_packages = list()
        colcon_packages_ignore = list()
        ros_build_snaps = list()

    plugin = colcon.ColconPlugin(part_name="my-part", options=Options())

    monkeypatch.setattr(sys, "path", ["", "/test"])
    monkeypatch.setattr(sys, "executable", "/test/python3")
    monkeypatch.setattr(_ros, "__file__", "/test/_ros.py")
    monkeypatch.setattr(os, "environ", dict())

    assert plugin.get_build_commands() == [
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then",
        "sudo --preserve-env=http_proxy,https_proxy rosdep init; fi",
        'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/snap" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/opt/ros/snap" . "/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        'eval "${state}"',
        'rm -f "${SNAPCRAFT_PART_INSTALL}/.installed_packages.txt"',
        'rm -f "${SNAPCRAFT_PART_INSTALL}/.build_snaps.txt"',
        'rosdep install --default-yes --ignore-packages-from-source --from-paths "${SNAPCRAFT_PART_SRC_WORK}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/snap" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/opt/ros/snap" . "/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        'eval "${state}"',
        "## Build command",
        "colcon build "
        '--base-paths "${SNAPCRAFT_PART_SRC_WORK}" --build-base "${SNAPCRAFT_PART_BUILD}" '
        '--merge-install --install-base "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap '
        "--cmake-args -DCMAKE_BUILD_TYPE=Release "
        '--parallel-workers "${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        "## Post build command",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE ]; then',
        'rm "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE',
        "fi",
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 /test/python3 -I "
        "/test/_ros.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC_WORK}" --part-install "${SNAPCRAFT_PART_INSTALL}" '
        '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" --target-arch "${SNAPCRAFT_TARGET_ARCH}"',
    ]


def test_get_build_commands_with_all_properties(monkeypatch):
    class Options:
        colcon_ament_cmake_args = ["ament", "args..."]
        colcon_catkin_cmake_args = ["catkin", "args..."]
        colcon_cmake_args = ["cmake", "args..."]
        colcon_packages = ["package1", "package2..."]
        colcon_packages_ignore = ["ipackage1", "ipackage2..."]
        ros_build_snaps = ["foo"]

    plugin = colcon.ColconPlugin(part_name="my-part", options=Options())

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
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then",
        "sudo --preserve-env=http_proxy,https_proxy rosdep init; fi",
        'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/snap/foo/current/opt/ros/${ROS_DISTRO}" . "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/snap/foo/current/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/snap/foo/current/opt/ros/snap" . "/snap/foo/current/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/snap" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/opt/ros/snap" . "/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        'eval "${state}"',
        'rm -f "${SNAPCRAFT_PART_INSTALL}/.installed_packages.txt"',
        'rm -f "${SNAPCRAFT_PART_INSTALL}/.build_snaps.txt"',
        "if [ -d /snap/foo/current/opt/ros ]; then",
        "ROS_PACKAGE_PATH=/snap/foo/current/opt/ros rospack list-names | (xargs "
        'rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | awk '
        '"/#apt/{getline;print;}" >> '
        '"${SNAPCRAFT_PART_INSTALL}/.installed_packages.txt"',
        "fi",
        'if [ -d "/snap/foo/current/opt/ros/${ROS_DISTRO}/" ]; then',
        'rosdep keys --rosdistro "${ROS_DISTRO}" --from-paths '
        '"/snap/foo/current/opt/ros/${ROS_DISTRO}" --ignore-packages-from-source | '
        '(xargs rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | grep -v "#" '
        '>> "${SNAPCRAFT_PART_INSTALL}"/.installed_packages.txt',
        "fi",
        'if [ -d "/snap/foo/current/opt/ros/snap/" ]; then',
        'rosdep keys --rosdistro "${ROS_DISTRO}" --from-paths '
        '"/snap/foo/current/opt/ros/snap" --ignore-packages-from-source | (xargs '
        'rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | grep -v "#" >> '
        '"${SNAPCRAFT_PART_INSTALL}"/.installed_packages.txt',
        "fi",
        "",
        'rosdep install --default-yes --ignore-packages-from-source --from-paths "${SNAPCRAFT_PART_SRC_WORK}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/snap/foo/current/opt/ros/${ROS_DISTRO}" . "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/snap/foo/current/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/snap/foo/current/opt/ros/snap" . "/snap/foo/current/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/snap" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/opt/ros/snap" . "/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        'eval "${state}"',
        "## Build command",
        "colcon build "
        '--base-paths "${SNAPCRAFT_PART_SRC_WORK}" --build-base "${SNAPCRAFT_PART_BUILD}" '
        '--merge-install --install-base "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap '
        "--packages-ignore ipackage1 ipackage2... --packages-select package1 "
        "package2... --cmake-args -DCMAKE_BUILD_TYPE=Release cmake args... "
        "--ament-cmake-args ament args... --catkin-cmake-args catkin "
        'args... --parallel-workers "${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        "## Post build command",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE ]; then',
        'rm "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE',
        "fi",
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 PATH=/bin:/test SNAP=TESTSNAP "
        "SNAP_ARCH=TESTARCH SNAP_NAME=TESTSNAPNAME SNAP_VERSION=TESTV1 "
        "http_proxy=http://foo https_proxy=https://bar "
        "/test/python3 -I /test/_ros.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC_WORK}" --part-install "${SNAPCRAFT_PART_INSTALL}" '
        '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" --target-arch "${SNAPCRAFT_TARGET_ARCH}"',
    ]


def test_get_build_commands_with_cmake_debug(monkeypatch):
    class Options:
        colcon_ament_cmake_args = ["ament", "args..."]
        colcon_catkin_cmake_args = ["catkin", "args..."]
        colcon_cmake_args = ["-DCMAKE_BUILD_TYPE=Debug", "args..."]
        colcon_packages = ["package1", "package2..."]
        colcon_packages_ignore = ["ipackage1", "ipackage2..."]
        ros_build_snaps = ["foo"]

    plugin = colcon.ColconPlugin(part_name="my-part", options=Options())

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
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then",
        "sudo --preserve-env=http_proxy,https_proxy rosdep init; fi",
        'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/snap/foo/current/opt/ros/${ROS_DISTRO}" . "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/snap/foo/current/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/snap/foo/current/opt/ros/snap" . "/snap/foo/current/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/snap" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/opt/ros/snap" . "/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        'eval "${state}"',
        'rm -f "${SNAPCRAFT_PART_INSTALL}/.installed_packages.txt"',
        'rm -f "${SNAPCRAFT_PART_INSTALL}/.build_snaps.txt"',
        "if [ -d /snap/foo/current/opt/ros ]; then",
        "ROS_PACKAGE_PATH=/snap/foo/current/opt/ros rospack list-names | (xargs "
        'rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | awk '
        '"/#apt/{getline;print;}" >> '
        '"${SNAPCRAFT_PART_INSTALL}/.installed_packages.txt"',
        "fi",
        'if [ -d "/snap/foo/current/opt/ros/${ROS_DISTRO}/" ]; then',
        'rosdep keys --rosdistro "${ROS_DISTRO}" --from-paths '
        '"/snap/foo/current/opt/ros/${ROS_DISTRO}" --ignore-packages-from-source | '
        '(xargs rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | grep -v "#" '
        '>> "${SNAPCRAFT_PART_INSTALL}"/.installed_packages.txt',
        "fi",
        'if [ -d "/snap/foo/current/opt/ros/snap/" ]; then',
        'rosdep keys --rosdistro "${ROS_DISTRO}" --from-paths '
        '"/snap/foo/current/opt/ros/snap" --ignore-packages-from-source | (xargs '
        'rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | grep -v "#" >> '
        '"${SNAPCRAFT_PART_INSTALL}"/.installed_packages.txt',
        "fi",
        "",
        'rosdep install --default-yes --ignore-packages-from-source --from-paths "${SNAPCRAFT_PART_SRC_WORK}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/snap/foo/current/opt/ros/${ROS_DISTRO}" . "/snap/foo/current/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/snap/foo/current/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/snap/foo/current/opt/ros/snap" . "/snap/foo/current/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="${SNAPCRAFT_PART_INSTALL}/opt/ros/snap" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
        'AMENT_CURRENT_PREFIX="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
        "fi",
        'if [ -f "/opt/ros/snap/local_setup.sh" ]; then',
        'COLCON_CURRENT_PREFIX="/opt/ros/snap" . "/opt/ros/snap/local_setup.sh"',
        "fi",
        "",
        'eval "${state}"',
        "## Build command",
        "colcon build "
        '--base-paths "${SNAPCRAFT_PART_SRC_WORK}" --build-base "${SNAPCRAFT_PART_BUILD}" '
        '--merge-install --install-base "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap '
        "--packages-ignore ipackage1 ipackage2... --packages-select package1 "
        "package2... --cmake-args -DCMAKE_BUILD_TYPE=Debug args... "
        "--ament-cmake-args ament args... --catkin-cmake-args catkin "
        'args... --parallel-workers "${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        "## Post build command",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE ]; then',
        'rm "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE',
        "fi",
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 PATH=/bin:/test SNAP=TESTSNAP "
        "SNAP_ARCH=TESTARCH SNAP_NAME=TESTSNAPNAME SNAP_VERSION=TESTV1 "
        "http_proxy=http://foo https_proxy=https://bar "
        "/test/python3 -I /test/_ros.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC_WORK}" --part-install "${SNAPCRAFT_PART_INSTALL}" '
        '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" --target-arch "${SNAPCRAFT_TARGET_ARCH}"',
    ]
