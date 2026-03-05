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
from typing_extensions import override

from snapcraft.const import SNAPCRAFT_ENVIRONMENT_VARIABLES


class Provider(ProviderService):
    """Snapcraft specialization of the Lifecycle Service."""

    @override
    def setup(self) -> None:
        """Add snapcraft environment variables to the build environment."""
        super().setup()
        for variable in SNAPCRAFT_ENVIRONMENT_VARIABLES:
            if variable in os.environ:
                self.environment[variable] = os.environ[variable]
