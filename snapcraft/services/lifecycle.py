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

from typing import cast

from craft_application import LifecycleService
from craft_parts import Features, callbacks, Step
from overrides import override

from snapcraft.projects import SnapcraftProject

from snapcraft.parts import lifecycle, plugins


class SnapcraftLifecycleService(LifecycleService):
    """Snapcraft-specific lifecycle service."""

    @override
    def setup(self) -> None:
        """Initialize the LifecycleManager with previously-set arguments."""
        plugins.register()

        callbacks.register_prologue(lifecycle._set_global_environment)
        callbacks.register_pre_step(lifecycle._set_step_environment)
        callbacks.register_post_step(lifecycle._patch_elf, step_list=[Step.PRIME])

        # Configure extra args to the LifecycleManager
        project = cast(SnapcraftProject, self._project)
        project_vars = {
            "version": project.version,
            "grade": project.grade or "",
        }

        self._manager_kwargs.update(
            base=project.base,
            package_repositories=project.package_repositories or [],
            project_name=project.name,
            project_vars=project_vars,
        )

        super().setup()

