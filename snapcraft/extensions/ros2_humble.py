# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

# Import types and tell flake8 to ignore the "unused" List.

"""Extension to the Colcon plugin for ROS 2 Humble."""

from typing import Any, Dict, Optional, Tuple

from overrides import overrides
from typing_extensions import Final

from .extension import Extension, get_extensions_data_dir


class ROS2HumbleExtension(Extension):
    """Drives ROS 2 build and runtime environment for snap."""

    ROS_VERSION: Final[str] = "2"
    ROS_DISTRO: Final[str] = "humble"

    @staticmethod
    @overrides
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core22",)

    @staticmethod
    @overrides
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    @staticmethod
    @overrides
    def is_experimental(base: Optional[str]) -> bool:
        return False

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        return {
            "package-repositories": [
                {
                    "type": "apt",
                    "url": "http://repo.ros2.org/ubuntu/main",
                    "components": ["main"],
                    "formats": ["deb"],
                    "key-id": "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                    "key-server": "keyserver.ubuntu.com",
                    "suites": ["jammy"],
                }
            ],
            "lint": {
                "ignore": [
                    {
                        "unused-library": [
                            "opt/ros/*",
                            "lib/*/libcrypt.so*",
                            "lib/*/libexpat.so*",
                            "lib/*/libtirpc.so*",
                            "lib/*/libz.so*",
                            "usr/lib/*libatomic.so*",
                            "usr/lib/*libconsole_bridge.so*",
                            "usr/lib/*libfmt.so*",
                            "usr/lib/*libicui18n.so*",
                            "usr/lib/*libicuio.so*",
                            "usr/lib/*libicutest.so*",
                            "usr/lib/*libicutu.so*",
                            "usr/lib/*libpython3.10.so*",
                            "usr/lib/*libspdlog.so*",
                            "usr/lib/*libtinyxml2.so*",
                        ]
                    }
                ]
            },
        }

    @overrides
    def get_app_snippet(self) -> Dict[str, Any]:
        python_paths = [
            f"$SNAP/opt/ros/{self.ROS_DISTRO}/lib/python3.10/site-packages",
            "$SNAP/usr/lib/python3/dist-packages",
            "${PYTHONPATH}",
        ]
        return {
            "command-chain": ["snap/command-chain/ros2-launch"],
            "environment": {
                "ROS_VERSION": self.ROS_VERSION,
                "ROS_DISTRO": self.ROS_DISTRO,
                "PYTHONPATH": ":".join(python_paths),
                # Various ROS 2 tools (e.g. spdlog logger) keep a cache or a log,
                # and use $ROS_HOME to determine where to put them.
                "ROS_HOME": "$SNAP_USER_DATA/ros",
            },
        }

    @overrides
    def get_part_snippet(self) -> Dict[str, Any]:
        return {
            "build-environment": [
                {"ROS_VERSION": self.ROS_VERSION},
                {"ROS_DISTRO": self.ROS_DISTRO},
            ]
        }

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        return {
            f"ros2-{self.ROS_DISTRO}/ros2-launch": {
                "source": f"{get_extensions_data_dir()}/ros2",
                "plugin": "nil",
                # pylint: disable=line-too-long
                "override-build": "install -D -m 0755 launch ${CRAFT_PART_INSTALL}/snap/command-chain/ros2-launch",
                "build-packages": [
                    f"ros-{self.ROS_DISTRO}-ros-environment",
                    f"ros-{self.ROS_DISTRO}-ros-workspace",
                    f"ros-{self.ROS_DISTRO}-ament-index-cpp",
                    f"ros-{self.ROS_DISTRO}-ament-index-python",
                ],
            }
        }
