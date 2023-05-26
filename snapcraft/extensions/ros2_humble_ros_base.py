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

"""Extension to the Colcon plugin for ROS 2 Humble using content sharing."""

from typing_extensions import Final

from ._ros2_humble_meta import ROS2HumbleMetaBase


class ROS2HumbleRosBaseExtension(ROS2HumbleMetaBase):
    """Drives ROS 2 build and runtime environment for snap using content-sharing."""

    ROS_META: Final[str] = "ros-humble-ros-base"
    ROS_META_DEV: Final[str] = "ros-humble-ros-base-dev"
    ROS_VARIANT: Final[str] = "ros-base"
