# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Extension to the Colcon plugin for ROS 2 Lyrical using content sharing."""

import functools

from overrides import overrides

from ._ros2_lyrical_meta import ROS2LyricalMetaBase, ROS2LyricalSnaps


class ROS2LyricalRosBaseExtension(ROS2LyricalMetaBase):
    """Drives ROS 2 build and runtime environment for snap using content-sharing."""

    @functools.cached_property
    @overrides
    def ros2_lyrical_snaps(  # type: ignore[reportIncompatibleMethodOverride]
        self,
    ) -> ROS2LyricalSnaps:
        return ROS2LyricalSnaps(
            sdk="ros-lyrical-ros-base-dev",
            content="ros-lyrical-ros-base",
            variant="ros-base",
        )
