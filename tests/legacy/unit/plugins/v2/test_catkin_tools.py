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
import snapcraft_legacy.plugins.v2.catkin_tools as catkin_tools


def test_schema():
    assert catkin_tools.CatkinToolsPlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "additionalProperties": False,
        "properties": {
            "catkin-tools-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
            },
            "catkin-tools-packages": {
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
            "ros-content-sharing-extension-cmake-args": {
                "type": "array",
                "minItems": 0,
                "items": {"type": "string"},
                "default": [],
            },
        },
        "type": "object",
    }


def test_get_build_packages():
    plugin = catkin_tools.CatkinToolsPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_packages() == {
        "python3-rosdep",
        "python3-catkin-tools",
        "rospack-tools",
    }


def test_get_build_snaps():
    class OptionsDefault:
        ros_build_snaps = list()

    plugin = catkin_tools.CatkinToolsPlugin(
        part_name="my-part", options=OptionsDefault()
    )

    assert plugin.get_build_snaps() == set()

    class Options:
        ros_build_snaps = ["Foo"]

    plugin = catkin_tools.CatkinToolsPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_snaps() == {"Foo"}


def test_get_build_environment():
    plugin = catkin_tools.CatkinToolsPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_environment() == {
        "ROS_PYTHON_VERSION": "3",
    }


def test_out_of_source_build_property():
    plugin = catkin_tools.CatkinToolsPlugin(part_name="my-part", options=lambda: None)

    assert plugin.out_of_source_build


def test_get_build_commands(monkeypatch):
    class Options:
        catkin_tools_cmake_args = list()
        catkin_tools_packages = list()
        ros_build_snaps = list()
        ros_content_sharing_extension_cmake_args = list()

    plugin = catkin_tools.CatkinToolsPlugin(part_name="my-part", options=Options())

    monkeypatch.setattr(sys, "path", ["", "/test"])
    monkeypatch.setattr(sys, "executable", "/test/python3")
    monkeypatch.setattr(_ros, "__file__", "/test/_ros.py")
    monkeypatch.setattr(os, "environ", dict())

    assert plugin.get_build_commands() == [
<<<<<<< HEAD
        'state="$(set +o); set -$-"',
        "set +u",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        "set -- --local",
        '_CATKIN_SETUP_DIR="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh"',
        "set -- --local --extend",
        "else",
        "set -- --local",
        "fi",
        '. /opt/ros/"${ROS_DISTRO}"/setup.sh',
        'eval "${state}"',
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then",
        "sudo --preserve-env=http_proxy,https_proxy rosdep init; fi",
=======
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep "
        "init; fi",
>>>>>>> 699b39a7 (ros: catkin_tools support for content sharing)
        'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . '
        '"${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . '
        '"/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
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
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . '
        '"${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . '
        '"/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        'eval "${state}"',
        "catkin init",
        "catkin profile add -f default",
        "catkin config --profile default --install "
        '--source-space "${SNAPCRAFT_PART_SRC_WORK}" --build-space "${SNAPCRAFT_PART_BUILD}" '
        '--install-space "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}"',
        'catkin build --no-notify --profile default -j "${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 /test/python3 -I "
        "/test/_ros.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC_WORK}" --part-install "${SNAPCRAFT_PART_INSTALL}" '
        '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" --target-arch "${SNAPCRAFT_TARGET_ARCH}"',
    ]


def test_get_build_commands_with_all_properties(monkeypatch):
    class Options:
        catkin_tools_cmake_args = ["cmake", "args..."]
        catkin_tools_packages = ["package1", "package2..."]
        ros_build_snaps = ["foo"]
        ros_content_sharing_extension_cmake_args = ["DCMAKE_EXNTENSION_ARGS", "bar"]

    plugin = catkin_tools.CatkinToolsPlugin(part_name="my-part", options=Options())

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
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        "set -- --local",
        '_CATKIN_SETUP_DIR="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh"',
        "set -- --local --extend",
        "else",
        "set -- --local",
        "fi",
        '. /opt/ros/"${ROS_DISTRO}"/setup.sh',
        'eval "${state}"',
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then",
        "sudo --preserve-env=http_proxy,https_proxy rosdep init; fi",
        'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/snap/foo/current/opt/ros/${ROS_DISTRO}" . '
        '"/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . '
        '"${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . '
        '"/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
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
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/snap/foo/current/opt/ros/${ROS_DISTRO}" . '
        '"/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        "## Sourcing ROS ws in stage snaps",
        'if [ -f "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" . "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        "## Sourcing ROS ws in system",
        'if [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . '
        '"/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        'eval "${state}"',
        "catkin init",
        "catkin profile add -f default",
        "catkin config --profile default --install "
        '--source-space "${SNAPCRAFT_PART_SRC_WORK}" --build-space "${SNAPCRAFT_PART_BUILD}" '
        '--install-space "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" --cmake-args cmake args... DCMAKE_EXNTENSION_ARGS bar',
        'catkin build --no-notify --profile default -j "${SNAPCRAFT_PARALLEL_BUILD_COUNT}" package1 package2...',
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 PATH=/bin:/test SNAP=TESTSNAP "
        "SNAP_ARCH=TESTARCH SNAP_NAME=TESTSNAPNAME SNAP_VERSION=TESTV1 "
        "http_proxy=http://foo https_proxy=https://bar "
        "/test/python3 -I "
        "/test/_ros.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC_WORK}" --part-install "${SNAPCRAFT_PART_INSTALL}" '
        '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" --target-arch "${SNAPCRAFT_TARGET_ARCH}"',
    ]
