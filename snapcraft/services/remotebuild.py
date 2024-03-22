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

from collections.abc import Collection
from typing import Any

from craft_application import launchpad
from craft_application.services import remotebuild
from overrides import override


class RemoteBuild(remotebuild.RemoteBuildService):
    """Snapcraft remote build service."""

    RecipeClass = launchpad.models.SnapRecipe

    @override
    def _new_recipe(
        self,
        name: str,
        repository: launchpad.models.GitRepository,
        architectures: Collection[str] | None = None,
        **_: Any,  # noqa: ANN401
    ) -> launchpad.models.Recipe:
        """Create a new recipe."""
        return launchpad.models.SnapRecipe.new(
            self.lp,
            name,
            self.lp.username,
            architectures=architectures,
            project=self._lp_project.name,
            git_ref=repository.git_https_url,
        )
