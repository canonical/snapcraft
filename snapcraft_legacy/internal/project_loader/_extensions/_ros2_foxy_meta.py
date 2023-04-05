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

from typing import Any, Dict

from .ros2_foxy import ExtensionImpl as RosFoxyExtension


class RosFoxyMetaBase(RosFoxyExtension):
    """Setup a ROS 2 build and runtime environment suitable for a snap."""

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        # python_paths = [
        #     f"$SNAP/opt/ros/{self.ROS_DISTRO}/lib/python3.8/site-packages",
        #     "$SNAP/usr/lib/python3/dist-packages",
        #     "${PYTHONPATH}",
        # ]

        # Very unlikely but it may happen that the snapped application doesn't
        # even pull those deps. In that case, there is no valid ROS 2 ws to source.
        # We make sure here that they are staged no matter what.
        self.parts[f"ros2-{self.ROS_DISTRO}-extension"]["stage-packages"] = [
            f"ros-{self.ROS_DISTRO}-ros-environment",
            f"ros-{self.ROS_DISTRO}-ros-workspace",
            f"ros-{self.ROS_DISTRO}-ament-index-cpp",
            f"ros-{self.ROS_DISTRO}-ament-index-python",
        ]

        self.root_snippet["plugs"] = {
            "ros-meta":
                {
                    "interface": "content",
                    "content": "ros2-meta",
                    "target": "$SNAP/opt/ros/underlay_ws",
                    "default-provider": self.ROS_META,
                }
        }

        self.part_snippet["build-snaps"] = [self.ROS_META_DEV]
