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

import functools

from overrides import overrides

from ._ros2_humble_meta import ROS2HumbleMetaBase, ROS2HumbleSnaps


class ROS2HumbleRosCoreExtension(ROS2HumbleMetaBase):
    """Drives ROS 2 build and runtime environment for snap using content-sharing."""

    @functools.cached_property  # type: ignore[reportIncompatibleMethodOverride]
    @overrides
    def ros2_humble_snaps(self) -> ROS2HumbleSnaps:
        return ROS2HumbleSnaps(
            sdk="ros-humble-ros-core-dev",
            content="ros-humble-ros-core",
            variant="ros-core",
        )
