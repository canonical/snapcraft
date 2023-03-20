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
import snapcraft_legacy.plugins.v2.catkin as catkin


def test_schema():
    assert catkin.CatkinPlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "additionalProperties": False,
        "properties": {
            "catkin-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
            },
            "catkin-packages": {
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
                "uniqueItems": True,
            },
            "catkin-packages-ignore": {
                "default": [],
                "items": {"type": "string"},
                "minItems": 0,
                "type": "array",
                "uniqueItems": True,
            },
        },
        "type": "object",
    }


def test_get_build_packages():
    plugin = catkin.CatkinPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_packages() == {"python3-rosdep", "ros-noetic-catkin"}


def test_get_build_environment():
    plugin = catkin.CatkinPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_environment() == {
        "ROS_PYTHON_VERSION": "3",
    }


def test_out_of_source_build_property():
    plugin = catkin.CatkinPlugin(part_name="my-part", options=lambda: None)

    assert plugin.out_of_source_build


def test_get_build_commands(monkeypatch):
    class Options:
        catkin_cmake_args = list()
        catkin_packages = list()
        catkin_packages_ignore = list()
        build_snaps = list()

    plugin = catkin.CatkinPlugin(part_name="my-part", options=Options())

    monkeypatch.setattr(sys, "path", ["", "/test"])
    monkeypatch.setattr(sys, "executable", "/test/python3")
    monkeypatch.setattr(_ros, "__file__", "/test/_ros.py")
    monkeypatch.setattr(os, "environ", dict())

    assert plugin.get_build_commands() == [
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep "
        "init; fi",
        'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
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
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        'eval "${state}"',
        'rosdep install --default-yes --ignore-packages-from-source --from-paths "${SNAPCRAFT_PART_SRC_WORK}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
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
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        'eval "${state}"',
        "## Build command",
        "catkin_make_isolated --install --merge "
        '--source-space "${SNAPCRAFT_PART_SRC_WORK}" --build-space "${SNAPCRAFT_PART_BUILD}" '
        '--install-space "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" '
        '-j "${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 /test/python3 -I "
        "/test/_ros.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC_WORK}" --part-install "${SNAPCRAFT_PART_INSTALL}" '
        '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" --target-arch "${SNAPCRAFT_TARGET_ARCH}"',
    ]


def test_get_build_commands_with_all_properties(monkeypatch):
    class Options:
        catkin_cmake_args = ["cmake", "args..."]
        catkin_packages = ["package1", "package2..."]
        catkin_packages_ignore = ["ipackage1", "ipackage2..."]
        build_snaps = ["foo"]

    plugin = catkin.CatkinPlugin(part_name="my-part", options=Options())

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
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep "
        "init; fi",
        'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/snap/foo/current/opt/ros/${ROS_DISTRO}" . "/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh"',
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
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        'eval "${state}"',
        'rosdep install --default-yes --ignore-packages-from-source --from-paths "${SNAPCRAFT_PART_SRC_WORK}"',
        'state="$(set +o); set -$-"',
        "set +u",
        "",
        "## Sourcing ROS ws in build snaps",
        'if [ -f "/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh" ]; then',
        'set -- --local "${_EXTEND_WS}"',
        '_CATKIN_SETUP_DIR="/snap/foo/current/opt/ros/${ROS_DISTRO}" . "/snap/foo/current/opt/ros/${ROS_DISTRO}/setup.sh"',
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
        '_CATKIN_SETUP_DIR="/opt/ros/${ROS_DISTRO}" . "/opt/ros/${ROS_DISTRO}/setup.sh"',
        'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
        "fi",
        "",
        'eval "${state}"',
        "## Build command",
        "catkin_make_isolated --install --merge "
        '--source-space "${SNAPCRAFT_PART_SRC_WORK}" --build-space "${SNAPCRAFT_PART_BUILD}" '
        '--install-space "${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}" '
        '-j "${SNAPCRAFT_PARALLEL_BUILD_COUNT}" --pkg package1 package2... '
        "--ignore-pkg ipackage1 ipackage2... --cmake-args cmake args...",
        "env -i LANG=C.UTF-8 LC_ALL=C.UTF-8 PATH=/bin:/test SNAP=TESTSNAP "
        "SNAP_ARCH=TESTARCH SNAP_NAME=TESTSNAPNAME SNAP_VERSION=TESTV1 "
        "http_proxy=http://foo https_proxy=https://bar "
        "/test/python3 -I "
        "/test/_ros.py "
        'stage-runtime-dependencies --part-src "${SNAPCRAFT_PART_SRC_WORK}" --part-install "${SNAPCRAFT_PART_INSTALL}" '
        '--ros-version "${ROS_VERSION}" --ros-distro "${ROS_DISTRO}" --target-arch "${SNAPCRAFT_TARGET_ARCH}"',
    ]
