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

from snapcraft_legacy.internal.project_loader._extensions.ros1_noetic_desktop import (
    ExtensionImpl as Ros1NoeticDesktopExtension,
)
from snapcraft_legacy.internal.project_loader._extensions.ros1_noetic_perception import (
    ExtensionImpl as Ros1NoeticPerceptionExtension,
)
from snapcraft_legacy.internal.project_loader._extensions.ros1_noetic_robot import (
    ExtensionImpl as Ros1NoeticRobotExtension,
)
from snapcraft_legacy.internal.project_loader._extensions.ros1_noetic_ros_base import (
    ExtensionImpl as Ros1NoeticRosBaseExtension,
)
from snapcraft_legacy.internal.project_loader._extensions.ros1_noetic_ros_core import (
    ExtensionImpl as Ros1NoeticRosCoreExtension,
)


class TestClass:
    scenarios = [
        (
            "desktop",
            {
                "extension_name": "ros1-noetic-desktop",
                "extension_class": Ros1NoeticDesktopExtension,
                "meta": "ros-noetic-desktop",
                "meta_dev": "ros-noetic-desktop-dev",
            },
            "perception",
            {
                "extension_name": "ros1-noetic-perception",
                "extension_class": Ros1NoeticPerceptionExtension,
                "meta": "ros-noetic-perception",
                "meta_dev": "ros-noetic-perception-dev",
            },
            "robot",
            {
                "extension_name": "ros1-noetic-robot",
                "extension_class": Ros1NoeticRobotExtension,
                "meta": "ros-noetic-robot",
                "meta_dev": "ros-noetic-robot-dev",
            },
            "ros-base",
            {
                "extension_name": "ros1-noetic-ros-base",
                "extension_class": Ros1NoeticRosBaseExtension,
                "meta": "ros-noetic-ros-base",
                "meta_dev": "ros-noetic-ros-base-dev",
            },
            "ros-core",
            {
                "extension_name": "ros1-noetic-ros-core",
                "extension_class": Ros1NoeticRosCoreExtension,
                "meta": "ros-meta-ros-core",
                "meta_dev": "ros-meta-ros-core-dev",
            },
        ),
    ]

    def test_extension(self, extension_name, extension_class, meta, meta_dev):
        ros1_extension = extension_class(
            extension_name=extension_name, yaml_data=dict(base="core20")
        )

        assert ros1_extension.is_experimental(None)

        assert ros1_extension.root_snippet == {
            "package-repositories": [
                {
                    "components": ["main"],
                    "formats": ["deb"],
                    "key-id": "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                    "key-server": "keyserver.ubuntu.com",
                    "suites": ["focal"],
                    "type": "apt",
                    "url": "http://packages.ros.org/ros/ubuntu",
                }
            ],
            "plugs": {
                "ros-noetic": {
                    "interface": "content",
                    "content": "ros-noetic",
                    "target": "$SNAP/opt/ros/underlay_ws",
                    "default-provider": meta,
                }
            },
        }

        python_paths = [
            "$SNAP/opt/ros/noetic/lib/python3.8/site-packages",
            "$SNAP/usr/lib/python3/dist-packages",
            "${PYTHONPATH}",
            "$SNAP/opt/ros/underlay_ws/opt/ros/noetic/lib/python3.8/site-packages",
            "$SNAP/opt/ros/underlay_ws/usr/lib/python3/dist-packages",
        ]

        assert ros1_extension.app_snippet == {
            "command-chain": ["snap/command-chain/ros1-launch"],
            "environment": {
                "PYTHONPATH": ":".join(python_paths),
                "ROS_VERSION": "1",
                "ROS_DISTRO": "noetic",
            },
        }

        assert ros1_extension.part_snippet == {
            "build-environment": [{"ROS_VERSION": "1"}, {"ROS_DISTRO": "noetic"}],
            "ros-build-snaps": [meta_dev],
            "ros-content-sharing-extension-cmake-args": [
                f'-DCMAKE_SYSTEM_PREFIX_PATH="/snap/{meta_dev}/current/usr"'
            ],
            "stage-packages": ["ros-noetic-ros-environment"],
        }

        assert ros1_extension.parts == {
            "ros1-noetic-extension": {
                "build-packages": ["ros-noetic-catkin"],
                "plugin": "make",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/ros1",
            }
        }

    def test_supported_bases(self, extension_name, extension_class, meta, meta_dev):
        assert extension_class.get_supported_bases() == ("core20",)

    def test_supported_confinement(
        self, extension_name, extension_class, meta, meta_dev
    ):
        extension_class.get_supported_confinement() == ("strict", "devmode")
