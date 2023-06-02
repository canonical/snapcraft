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

# Import types and tell flake8 to ignore the "unused" List.

import dataclasses
from abc import abstractmethod
from typing import Any, Dict, Optional

from overrides import overrides

from .ros1_noetic import ExtensionImpl as RosNoeticExtension


@dataclasses.dataclass
class ROS2NoeticSnaps:
    """A structure of ROS 2 Noetic related snaps."""

    sdk: str
    content: str


class RosNoeticMetaBase(RosNoeticExtension):
    """Setup a ROS 1 build and runtime environment suitable for a snap."""

    @property
    @abstractmethod
    def ros_noetic_snaps(self) -> ROS2NoeticSnaps:
        """Return the ROS Noetic related snaps to use to construct the environment."""
        raise NotImplementedError

    @staticmethod
    @overrides
    def is_experimental(base: Optional[str]) -> bool:
        return True

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        self.root_snippet["plugs"] = {
            "ros-noetic":
                {
                    "interface": "content",
                    "content": "ros-noetic",
                    "target": "$SNAP/opt/ros/underlay_ws",
                    "default-provider": self.ros_noetic_snaps.content,
                }
        }

        self.part_snippet["catkin-cmake-args"] = [
            f'-DCMAKE_SYSTEM_PREFIX_PATH="/snap/{self.ros_noetic_snaps.sdk}/current/usr"'
        ]

        self.part_snippet["stage-packages"] = [f"ros-{self.ROS_DISTRO}-ros-environment"]

        self.part_snippet["ros-build-snaps"] = [self.ros_noetic_snaps.sdk]

        python_paths = self.app_snippet["environment"]["PYTHONPATH"]
        new_python_paths = [
            f"$SNAP/opt/ros/underlay_ws/opt/ros/{self.ROS_DISTRO}/lib/python3.8/site-packages",
            "$SNAP/opt/ros/underlay_ws/usr/lib/python3/dist-packages",
        ]

        self.app_snippet["environment"]["PYTHONPATH"] = f'{python_paths}:{":".join(new_python_paths)}'

        self.app_snippet["environment"]["PATH"] = "$SNAP/opt/ros/underlay_ws/usr/bin:$PATH"
        self.app_snippet["environment"]["LD_LIBRARY_PATH"] = "$SNAP/opt/ros/underlay_ws/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:$LD_LIBRARY_PATH"

        # @todo: for some reason ${VAR:+:$VAR} resulsts in empty substitution
        # self.app_snippet["environment"]["PATH"] = "$SNAP/opt/ros/underlay_ws/usr/bin${PATH:+:$PATH}"
        # self.app_snippet["environment"]["LD_LIBRARY_PATH"] = "$SNAP/opt/ros/underlay_ws/usr/lib/$SNAPCRAFT_ARCH_TRIPLET${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
