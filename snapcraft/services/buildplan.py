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

from collections.abc import Collection, Iterable, Sequence
from typing import Literal

import craft_cli
import craft_platforms
from craft_application.services.buildplan import BuildPlanService


class BuildPlan(BuildPlanService):
    """Snapcraft-specific build plan service."""

    def _filter_launchpad_plan(
        self,
        /,
        exhaustive_build_plan: Iterable[craft_platforms.BuildInfo],
        *,
        platforms: Collection[str] | None,
        build_for: Collection[craft_platforms.DebianArchitecture | Literal["all"]]
        | None,
        build_on: Collection[craft_platforms.DebianArchitecture] | None,
    ) -> Iterable[craft_platforms.BuildInfo]:
        """Filter the build plan.

        This method filters the given build plan based on the provided filter values.
        An application may override this if in needs to filter the build plan in
        non-default ways. It exists to allow applications to only change the build plan
        filter and should only be used by :meth:`get_build_plan` except in testing.

        :param platforms: A collection of platform names to keep after filtering, or
            ``None`` to not filter on the platform name.
        :param build_for: A collection of target architectures to keep after filtering,
            or ``None`` to not filter on the build-for architecture.
        :param build_on: A collection of build-on architectures to keep after
            filtering, or ``None`` to not filter on the build-on architecture.
        :yields: Build info objects based on the given filter.
        """
        for item in exhaustive_build_plan:
            if platforms is not None and item.platform not in platforms:
                continue
            if build_for is not None and item.build_for not in build_for:
                continue
            if build_on is not None and item.build_on not in build_on:
                continue
            yield item

    def create_launchpad_build_plan(
        self,
        *,
        platforms: Collection[str] | None,
        build_for: Collection[str | craft_platforms.DebianArchitecture] | None,
        build_on: Collection[str | craft_platforms.DebianArchitecture] | None,
    ) -> Sequence[craft_platforms.BuildInfo]:
        """Generate a build plan based on the given platforms, build-fors and build-ons.

        If no platforms, build-fors or build-ons are provided, the method will
        provide all possible builds.

        This is different from the upstream `create_build_plan()` method, which
        requires some sort of filtering with build-on, build-for, or platform to
        creates a locally actionable build plan that has no more than one build item
        for each platform.

        :param platforms: A collection of platform names to select, or None to not
            filter on platform names.
        :param build_for: A collection of build-for architecture names to select, or
            None to not filter on build-for architectures. Defaults to None.
        :param build_on: A collection of build-on architecture names to select.
            Defaults to None.

        :returns: A build plan for the given platforms, build-fors and build-ons.
        """
        project_service = self._services.get("project")
        raw_project = project_service.get_raw()
        raw_project["platforms"] = project_service.get_platforms()

        if build_on:
            build_on_archs = [craft_platforms.DebianArchitecture(on) for on in build_on]
        else:
            build_on_archs = None

        if build_for:
            build_for_archs = [
                "all" if fr == "all" else craft_platforms.DebianArchitecture(fr)
                for fr in build_for
            ]
        else:
            build_for_archs = None

        plan = list(
            self._filter_launchpad_plan(
                self._gen_exhaustive_build_plan(project_data=raw_project),
                platforms=platforms,
                build_for=build_for_archs,  # type: ignore[arg-type]  # Literal "all"
                build_on=build_on_archs,
            )
        )

        craft_cli.emit.debug(f"Build plan contains {len(plan)} build(s).")
        craft_cli.emit.trace(f"Build plan: {str(plan)}")

        return plan
