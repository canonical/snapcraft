# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Base for ROS 2 Humble extensions to the Colcon plugin using content-sharing."""

from abc import abstractmethod
from typing import Any, Dict

from overrides import overrides

from .ros2_humble import ROS2HumbleExtension


class ROS2HumbleMetaBase(ROS2HumbleExtension):
    """Drives ROS 2 build and runtime environment for snap using content-sharing."""

    @property
    @abstractmethod
    def ROS_META(self):
        """Abstract property to define the extension's content-sharing snap."""
        raise NotImplementedError

    @property
    @abstractmethod
    def ROS_META_DEV(self):
        """Abstract property to define the extension's build snap."""
        raise NotImplementedError

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        root_snippet = super().get_root_snippet()
        root_snippet["plugs"] = {
            "ros-humble": {
                "interface": "content",
                "content": "ros-humble",
                "target": "$SNAP/opt/ros/underlay_ws",
                "default-provider": self.ROS_META,
            }
        }
        return root_snippet

    @overrides
    def get_app_snippet(self) -> Dict[str, Any]:
        app_snippet = super().get_app_snippet()
        python_paths = app_snippet["environment"]["PYTHONPATH"]
        new_python_paths = [
            f"$SNAP/opt/ros/underlay_ws/opt/ros/{self.ROS_DISTRO}/lib/python3.8/site-packages",
            f"$SNAP/opt/ros/underlay_ws/usr/lib/python3/dist-packages",
        ]

        app_snippet["environment"]["PYTHONPATH"] = f'{python_paths}:{":".join(new_python_paths)}'

        return app_snippet

    @overrides
    def get_part_snippet(self) -> Dict[str, Any]:
        part_snippet = super().get_part_snippet()
        part_snippet["build-snaps"] = [self.ROS_META_DEV]
        return part_snippet

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        parts_snippet = super().get_parts_snippet()
        # Very unlikely but it may happen that the snapped application doesn't
        # even pull those deps. In that case, there is no valid ROS 2 ws to source.
        # We make sure here that they are staged no matter what.
        print("parts_snippet ", parts_snippet)
        parts_snippet[f"ros2-{self.ROS_DISTRO}/ros2-launch"]["stage-packages"] = [
            f"ros-{self.ROS_DISTRO}-ros-environment",
            f"ros-{self.ROS_DISTRO}-ros-workspace",
            f"ros-{self.ROS_DISTRO}-ament-index-cpp",
            f"ros-{self.ROS_DISTRO}-ament-index-python",
        ]
        return parts_snippet
