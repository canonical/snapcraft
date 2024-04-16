# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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
"""Snapcraft Lifecycle Service."""

import os

from craft_application import ProviderService
from overrides import overrides


class Provider(ProviderService):
    """Snapcraft specialization of the Lifecycle Service."""

    @overrides
    def setup(self) -> None:
        if build_info := os.getenv("SNAPCRAFT_BUILD_INFO"):
            self.environment["SNAPCRAFT_BUILD_INFO"] = build_info
        if image_info := os.getenv("SNAPCRAFT_IMAGE_INFO"):
            self.environment["SNAPCRAFT_IMAGE_INFO"] = image_info
        if experimental_extensions := os.getenv(
            "SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS"
        ):
            self.environment["SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS"] = (
                experimental_extensions
            )
