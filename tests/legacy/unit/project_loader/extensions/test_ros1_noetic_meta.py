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
                "extension_class": Ros1NoeticDesktopExtension,
                "meta": "ros-meta-desktop",
                "meta_dev": "ros-meta-desktop",
            },
            "perception",
            {
                "extension_class": Ros1NoeticPerceptionExtension,
                "meta": "ros-meta-perception",
                "meta_dev": "ros-meta-perception",
            },
            "robot",
            {
                "extension_class": Ros1NoeticRobotExtension,
                "meta": "ros-meta-robot",
                "meta_dev": "ros-meta-robot",
            },
            "ros-base",
            {
                "extension_class": Ros1NoeticRosBaseExtension,
                "meta": "ros-meta-ros-base",
                "meta_dev": "ros-meta-ros-base",
            },
            "ros-core",
            {
                "extension_class": Ros1NoeticRosCoreExtension,
                "meta": "ros-meta-ros-core",
                "meta_dev": "ros-meta-ros-core",
            },
        ),
    ]

    def test_extension(self, extension_class, meta, meta_dev):
        ros1_extension = extension_class(
            extension_name="ros1-noetic", yaml_data=dict(base="core20")
        )

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
                "ros-meta": {
                    "interface": "content",
                    "content": "ros-meta",
                    "target": "$SNAP/opt/ros/underlay_ws",
                    "default-provider": meta,
                }
            },
        }

        assert ros1_extension.app_snippet == {
            "command-chain": ["snap/command-chain/ros1-launch"],
            "environment": {
                "PYTHONPATH": "$SNAP/opt/ros/noetic/lib/python3.8/site-packages:$SNAP/usr/lib/python3/dist-packages:${PYTHONPATH}",
                "ROS_VERSION": "1",
                "ROS_DISTRO": "noetic",
            },
        }

        assert ros1_extension.part_snippet == {
            "build-environment": [{"ROS_VERSION": "1"}, {"ROS_DISTRO": "noetic"}],
            "build-snaps": [meta_dev],
        }

        assert ros1_extension.parts == {
            "ros1-noetic-extension": {
                "build-packages": ["ros-noetic-catkin"],
                "override-build": "install -D -m 0755 launch "
                "${SNAPCRAFT_PART_INSTALL}/snap/command-chain/ros1-launch",
                "plugin": "nil",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/ros1",
            }
        }

    def test_supported_bases(self, extension_class, meta, meta_dev):
        assert extension_class.get_supported_bases() == ("core20",)

    def test_supported_confinement(self, extension_class, meta, meta_dev):
        extension_class.get_supported_confinement() == ("strict", "devmode")
