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
"""Snapcraft Lifecycle service."""

from __future__ import annotations

from craft_application import LifecycleService
from craft_parts import Step, callbacks
from overrides import override

from snapcraft import models
from snapcraft.parts import lifecycle, plugins


class Lifecycle(LifecycleService):
    """Snapcraft-specific lifecycle service."""

    _project: models.Project

    @override
    def setup(self) -> None:
        """Initialize the LifecycleManager with previously-set arguments."""
        plugins.register()

        callbacks.register_prologue(lifecycle.set_global_environment)
        callbacks.register_pre_step(lifecycle.set_step_environment)
        callbacks.register_post_step(lifecycle.patch_elf, step_list=[Step.PRIME])

        # Configure extra args to the LifecycleManager
        project_vars = {
            "version": self._project.version,
            "grade": self._project.grade or "",
        }

        self._manager_kwargs.update(
            base=self._project.base,
            package_repositories=self._project.package_repositories or [],
            project_name=self._project.name,
            project_vars=project_vars,
            project_base=self._project.base,
            confinement=self._project.confinement,
        )

        super().setup()
