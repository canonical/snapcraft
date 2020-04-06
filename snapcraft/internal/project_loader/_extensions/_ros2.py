# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

from abc import ABC, abstractmethod
from typing import Any, Dict, Tuple

from ._extension import Extension


class Ros2Extension(Extension, ABC):
    """This extension eases creation of snaps that integrate with ROS2."""

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        # TODO: core20
        return ("core18",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    @classmethod
    @abstractmethod
    def get_distro(cls) -> str:
        ...

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        self.root_snippet = {"layout": {"/opt": {"bind": "$SNAP/opt"}}}

        self.app_snippet = {
            "command-chain": ["snap/command-chain/ros2-launch"],
            "environment": {"SNAP_ROS_DISTRO": self.get_distro()},
        }

        self.parts = {
            "ros2-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/ros2",
                "plugin": "nil",
                "override-build": "snapcraftctl build && install -D -m 0755 ros2-launch $SNAPCRAFT_PART_INSTALL/snap/command-chain/ros2-launch",
            }
        }
