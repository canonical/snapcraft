# Copyright 2025 Canonical Ltd.
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

"""Snapcraft build plan service."""

from collections.abc import Sequence

import craft_cli
import craft_platforms
from craft_application.services.buildplan import BuildPlanService


class BuildPlan(BuildPlanService):
    """Snapcraft-specific build plan service."""

    def create_unfiltered_build_plan(self) -> Sequence[craft_platforms.BuildInfo]:
        """Create an unfiltered build plan.

        This is different from the upstream `create_build_plan()` method, which
        requires some sort of filtering with build-on, build-for, or platform to
        creates an actionable build plan that has no more than one build item
        for each platform.

        This method does no filtering and is useful in scenarios like remote build
        orchestration where all possible builds must be considered.

        :return: An unfiltered build plan for the project.
        """
        project_service = self._services.get("project")
        raw_project = project_service.get_raw()
        raw_project["platforms"] = project_service.get_platforms()

        plan = list(self._gen_exhaustive_build_plan(project_data=raw_project))

        craft_cli.emit.debug(f"Build plan contains {len(plan)} build(s).")
        craft_cli.emit.trace(f"Build plan: {str(plan)}")

        return plan
