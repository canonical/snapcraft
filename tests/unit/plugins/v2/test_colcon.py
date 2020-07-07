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

from textwrap import dedent

from snapcraft.plugins.v2.colcon import ColconPlugin


def test_schema():
    assert ColconPlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "additionalProperties": False,
        "properties": {
            "colcon-ament-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minitems": 1,
                "type": "array",
            },
            "colcon-catkin-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minitems": 1,
                "type": "array",
            },
            "colcon-cmake-args": {
                "default": [],
                "items": {"type": "string"},
                "minitems": 1,
                "type": "array",
            },
            "colcon-packages": {
                "items": {"type": "string"},
                "minitems": 1,
                "type": "array",
                "uniqueItems": True,
            },
            "colcon-packages-ignore": {
                "default": [],
                "items": {"type": "string"},
                "minitems": 1,
                "type": "array",
                "uniqueItems": True,
            },
        },
        "type": "object",
    }


def test_get_build_packages():
    plugin = ColconPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_packages() == {
        "python3-colcon-common-extensions",
        "python3-rosdep",
        "python3-rosinstall",
        "python3-wstool",
    }


def test_get_build_environment():
    plugin = ColconPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_environment() == {
        "AMENT_PYTHON_EXECUTABLE": "/usr/bin/python3",
        "COLCON_PYTHON_EXECUTABLE": "/usr/bin/python3",
        "ROS_PYTHON_VERSION": "3",
    }


def test_get_build_commands():
    class Options:
        colcon_ament_cmake_args = list()
        colcon_catkin_cmake_args = list()
        colcon_cmake_args = list()
        colcon_packages = list()
        colcon_packages_ignore = list()

    plugin = ColconPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        ". /opt/ros/$ROS_DISTRO/setup.sh",
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep "
        "init; fi",
        "rosdep update --include-eol-distros --rosdistro $ROS_DISTRO",
        "rosdep install --from-paths . --default-yes --ignore-packages-from-source",
        "colcon build --merge-install --install-base $SNAPCRAFT_PART_INSTALL "
        "--parallel-workers ${SNAPCRAFT_PARALLEL_BUILD_COUNT}",
        "env -i "
        "PATH=/home/ubuntu/.venv/snapcraft/bin:/home/ubuntu/.vscode-server/bin/cd9ea6488829f560dc949a8b2fb789f3cdc05f5d/bin:/home/ubuntu/.local/bin:/home/ubuntu/.venv/snapcraft/bin:/home/ubuntu/.vscode-server/bin/cd9ea6488829f560dc949a8b2fb789f3cdc05f5d/bin:/home/ubuntu/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin:/var/lib/snapd/snap/bin:/snap/bin:/var/lib/snapd/snap/bin:/snap/bin:/var/lib/snapd/snap/bin "
        "LC_ALL=C.UTF-8 LANG=C.UTF-8 "
        "PYTHONPATH=/home/ubuntu/snapcraft/tests/unit/plugins/v2:/home/ubuntu/snapcraft:/home/ubuntu/.venv/snapcraft/bin:/usr/lib/python36.zip:/usr/lib/python3.6:/usr/lib/python3.6/lib-dynload:/home/ubuntu/.venv/snapcraft/lib/python3.6/site-packages:/home/ubuntu/snapcraft "
        "/home/ubuntu/.venv/snapcraft/bin/python3 -S "
        "/home/ubuntu/snapcraft/snapcraft/plugins/v2/colcon.py "
        "stage-runtime-dependencies --part-install $SNAPCRAFT_PART_INSTALL "
        "--ros-distro $ROS_DISTRO",
    ]


def test_get_build_commands_with_all_properties():
    class Options:
        colcon_ament_cmake_args = ["ament", "args..."]
        colcon_catkin_cmake_args = ["catkin", "args..."]
        colcon_cmake_args = ["cmake", "args..."]
        colcon_packages = ["package1", "package2..."]
        colcon_packages_ignore = ["ipackage1", "ipackage2..."]

    plugin = ColconPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == [
        ". /opt/ros/$ROS_DISTRO/setup.sh",
        "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep "
        "init; fi",
        "rosdep update --include-eol-distros --rosdistro $ROS_DISTRO",
        "rosdep install --from-paths . --default-yes --ignore-packages-from-source",
        "colcon build --merge-install --install-base $SNAPCRAFT_PART_INSTALL "
        "--packages-ignore ipackage1 ipackage2... --packages-select package1 "
        "package2... --ament-cmake-args ament args... --catkin-cmake-args catkin "
        "args... --parallel-workers ${SNAPCRAFT_PARALLEL_BUILD_COUNT}",
        "env -i "
        "PATH=/home/ubuntu/.venv/snapcraft/bin:/home/ubuntu/.vscode-server/bin/cd9ea6488829f560dc949a8b2fb789f3cdc05f5d/bin:/home/ubuntu/.local/bin:/home/ubuntu/.venv/snapcraft/bin:/home/ubuntu/.vscode-server/bin/cd9ea6488829f560dc949a8b2fb789f3cdc05f5d/bin:/home/ubuntu/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin:/var/lib/snapd/snap/bin:/snap/bin:/var/lib/snapd/snap/bin:/snap/bin:/var/lib/snapd/snap/bin "
        "LC_ALL=C.UTF-8 LANG=C.UTF-8 "
        "PYTHONPATH=/home/ubuntu/snapcraft/tests/unit/plugins/v2:/home/ubuntu/snapcraft:/home/ubuntu/.venv/snapcraft/bin:/usr/lib/python36.zip:/usr/lib/python3.6:/usr/lib/python3.6/lib-dynload:/home/ubuntu/.venv/snapcraft/lib/python3.6/site-packages:/home/ubuntu/snapcraft "
        "/home/ubuntu/.venv/snapcraft/bin/python3 -S "
        "/home/ubuntu/snapcraft/snapcraft/plugins/v2/colcon.py "
        "stage-runtime-dependencies --part-install $SNAPCRAFT_PART_INSTALL "
        "--ros-distro $ROS_DISTRO",
    ]
