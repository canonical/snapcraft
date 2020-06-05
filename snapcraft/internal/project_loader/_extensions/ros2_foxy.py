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

# Import types and tell flake8 to ignore the "unused" List.

from typing import Any, Dict, Optional, Tuple
from typing_extensions import Final

from ._extension import Extension

_PLATFORM_SNAP = dict(core18="gnome-3-34-1804")


class ExtensionImpl(Extension):
    """Drives ROS2 build and runtime environment for snap."""

    ROS_DISTRO: Final[str] = "foxy"

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core20",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    @staticmethod
    def is_experimental(base: Optional[str]) -> bool:
        return True

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        python_paths = [
            f"$SNAP/opt/ros/{self.ROS_DISTRO}/lib/python3.8/site-packages",
            "$SNAP/usr/lib/python3/dist-packages",
            "${PYTHONPATH}",
        ]

        self.root_snippet = {
            "package-repositories": [
                {
                    "type": "apt",
                    "url": "http://repo.ros2.org/ubuntu/main",
                    "components": ["main"],
                    "deb-types": ["deb"],
                    "key-id": "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                    "key-server": "keyserver.ubuntu.com",
                    "suites": ["$SNAPCRAFT_APT_RELEASE"],
                }
            ]
        }

        self.app_snippet = {
            "command-chain": ["snap/command-chain/ros2-launch"],
            "environment": {
                "ROS_DISTRO": self.ROS_DISTRO,
                "PYTHONPATH": ":".join(python_paths),
            },
        }

        self.part_snippet = {"build-environment": [{"ROS_DISTRO": self.ROS_DISTRO}]}

        self.parts = {
            f"ros2-{self.ROS_DISTRO}-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/ros2",
                "plugin": "nil",
                "override-build": "install -D -m 0755 launch ${SNAPCRAFT_PART_INSTALL}/snap/command-chain/ros2-launch",
                "build-packages": [f"ros-{self.ROS_DISTRO}-ros-core"],
            }
        }
