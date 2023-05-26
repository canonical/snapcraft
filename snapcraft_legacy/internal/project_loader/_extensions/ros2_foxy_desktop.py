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

import functools

from overrides import overrides

from ._ros2_foxy_meta import ROS2FoxySnaps, RosFoxyMetaBase


class ExtensionImpl(RosFoxyMetaBase):
    """Setup a ROS 2 build and runtime environment suitable for a snap."""

    @functools.cached_property
    @overrides
    def ros2_foxy_snaps(self) -> ROS2FoxySnaps:
        return ROS2FoxySnaps(
            content="ros-foxy-desktop", sdk="ros-foxy-desktop-dev"
        )
