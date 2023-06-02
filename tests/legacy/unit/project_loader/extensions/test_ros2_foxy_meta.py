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

from snapcraft_legacy.internal.project_loader._extensions.ros2_foxy_desktop import (
    ExtensionImpl as Ros2FoxyDesktopExtension,
)
from snapcraft_legacy.internal.project_loader._extensions.ros2_foxy_ros_base import (
    ExtensionImpl as Ros2FoxyRosBaseExtension,
)
from snapcraft_legacy.internal.project_loader._extensions.ros2_foxy_ros_core import (
    ExtensionImpl as Ros2FoxyRosCoreExtension,
)


class TestClass:
    scenarios = [
        (
            "desktop",
            {
                "extension_name": "ros2-foxy-desktop",
                "extension_class": Ros2FoxyDesktopExtension,
                "meta": "ros-foxy-desktop",
                "meta_dev": "ros-foxy-desktop-dev",
            },
            "ros-base",
            {
                "extension_name": "ros2-foxy-ros-base",
                "extension_class": Ros2FoxyRosBaseExtension,
                "meta": "ros-foxy-ros-base",
                "meta_dev": "ros-foxy-ros-base-dev",
            },
            "ros-core",
            {
                "extension_name": "ros2-foxy-ros-core",
                "extension_class": Ros2FoxyRosCoreExtension,
                "meta": "ros-foxy-ros-core",
                "meta_dev": "ros-foxy-ros-core-dev",
            },
        ),
    ]

    def test_extension(self, extension_name, extension_class, meta, meta_dev):
        ros_extension = extension_class(
            extension_name=extension_name, yaml_data=dict(base="core20")
        )

        assert ros_extension.is_experimental(None)

        assert ros_extension.root_snippet == {
            "package-repositories": [
                {
                    "components": ["main"],
                    "formats": ["deb"],
                    "key-id": "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                    "key-server": "keyserver.ubuntu.com",
                    "suites": ["focal"],
                    "type": "apt",
                    "url": "http://repo.ros2.org/ubuntu/main",
                }
            ],
            "plugs": {
                "ros-foxy": {
                    "interface": "content",
                    "content": "ros-foxy",
                    "target": "$SNAP/opt/ros/underlay_ws",
                    "default-provider": meta,
                }
            },
        }

        python_paths = [
            "$SNAP/opt/ros/foxy/lib/python3.8/site-packages",
            "$SNAP/usr/lib/python3/dist-packages",
            "${PYTHONPATH}",
            "$SNAP/opt/ros/underlay_ws/opt/ros/foxy/lib/python3.8/site-packages",
            "$SNAP/opt/ros/underlay_ws/usr/lib/python3/dist-packages",
        ]

        assert ros_extension.app_snippet == {
            "command-chain": ["snap/command-chain/ros2-launch"],
            "environment": {
                "LD_LIBRARY_PATH": "$SNAP/opt/ros/underlay_ws/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:$LD_LIBRARY_PATH",
                "PATH": "$SNAP/opt/ros/underlay_ws/usr/bin:$PATH",
                "PYTHONPATH": ":".join(python_paths),
                "ROS_VERSION": "2",
                "ROS_DISTRO": "foxy",
            },
        }

        assert ros_extension.part_snippet == {
            "build-environment": [{"ROS_VERSION": "2"}, {"ROS_DISTRO": "foxy"}],
            "ros-build-snaps": [meta_dev],
            "colcon-cmake-args": [
                f'-DCMAKE_SYSTEM_PREFIX_PATH="/snap/{meta_dev}/current/usr"'
            ],
        }

        assert ros_extension.parts == {
            "ros2-foxy-extension": {
                "build-packages": [
                    "ros-foxy-ros-environment",
                    "ros-foxy-ros-workspace",
                    "ros-foxy-ament-index-cpp",
                    "ros-foxy-ament-index-python",
                ],
                "stage-packages": [
                    "ros-foxy-ros-environment",
                    "ros-foxy-ros-workspace",
                    "ros-foxy-ament-index-cpp",
                    "ros-foxy-ament-index-python",
                ],
                "plugin": "make",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/ros2",
            }
        }

    def test_supported_bases(self, extension_name, extension_class, meta, meta_dev):
        assert extension_class.get_supported_bases() == ("core20",)

    def test_supported_confinement(
        self, extension_name, extension_class, meta, meta_dev
    ):
        extension_class.get_supported_confinement() == ("strict", "devmode")
