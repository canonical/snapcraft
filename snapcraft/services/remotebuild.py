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

from collections.abc import Collection, Mapping
from datetime import datetime
from pathlib import Path
from typing import Any

from craft_application import launchpad
from craft_application.services import remotebuild
from overrides import override


class RemoteBuild(remotebuild.RemoteBuildService):
    """Snapcraft remote build service."""

    RecipeClass = launchpad.models.SnapRecipe

    def fetch_logs(self, output_dir: Path) -> Mapping[str, Path | None]:
        """Fetch the logs for each build to the given directory.

        :param output_dir: The directory into which to place the logs.
        :returns: A mapping of the architecture to its build log.
        """
        if not self._is_setup:
            raise RuntimeError(
                "RemoteBuildService must be set up using start_builds "
                "or resume_builds before fetching logs."
            )
        project_name = self._name.split("-", maxsplit=2)[1]
        logs: dict[str, Path | None] = {}
        log_downloads: dict[str, Path] = {}
        fetch_time = datetime.now().isoformat(timespec="seconds")
        for build in self._builds:
            url = build.build_log_url
            if not url:
                logs[build.arch_tag] = None
                continue
            filename = (
                f"{project_name}_{build.distribution.name}-"
                f"{build.distro_series.version}-{build.arch_tag}-{fetch_time}.txt"
            )
            logs[build.arch_tag] = output_dir / filename
            log_downloads[url] = output_dir / filename
        self.request.download_files_with_progress(log_downloads)
        return logs

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
            git_ref=repository.self_link + "/+ref/main",
        )
