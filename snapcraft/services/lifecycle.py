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

from typing import cast

from craft_application import LifecycleService
from overrides import overrides

from snapcraft import models


class Lifecycle(LifecycleService):
    """Snapcraft specialization of the Lifecycle Service."""

    @overrides
    def setup(self) -> None:
        project = cast(models.Project, self._project)

        # Have the lifecycle install the base snap, and look into it when
        # determining the package cutoff.
        self._manager_kwargs.update(
            base=project.get_effective_base(),
            extra_build_snaps=project.get_extra_build_snaps(),
        )

        super().setup()
