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

"""Manager for creating, monitoring, and cleaning remote builds."""

import logging
from pathlib import Path
from typing import List, Optional

from .git import check_git_repo_for_remote_build
from .launchpad import LaunchpadClient
from .utils import get_build_id, humanize_list, validate_architectures
from .worktree import WorkTree

logger = logging.getLogger(__name__)


class RemoteBuilder:
    """Remote builder class.

    :param app_name: Name of the application.
    :param build_id: Unique identifier for the build.
    :param project_name: Name of the project.
    :param architectures: List of architectures to build on.
    :param project_dir: Path of the project.
    :param timeout: Time in seconds to wait for the build to complete.

    :raises UnsupportedArchitectureError: if any architecture is not supported
    for remote building.
    :raises LaunchpadHttpsError: If a connection to Launchpad cannot be established.
    """

    def __init__(  # noqa: PLR0913 pylint: disable=too-many-arguments
        self,
        app_name: str,
        build_id: Optional[str],
        project_name: str,
        architectures: List[str],
        project_dir: Path,
        timeout: int,
    ):
        self._app_name = app_name
        self._project_name = project_name
        self._project_dir = project_dir

        check_git_repo_for_remote_build(self._project_dir)

        if build_id:
            self._build_id = build_id
        else:
            self._build_id = get_build_id(
                app_name=self._app_name,
                project_name=self._project_name,
                project_path=self._project_dir,
            )

        validate_architectures(architectures)
        self._architectures = architectures

        self._worktree = WorkTree(
            app_name=self._app_name,
            build_id=self._build_id,
            project_dir=self._project_dir,
        )

        logger.debug("Setting up launchpad environment.")

        self._lpc = LaunchpadClient(
            app_name=self._app_name,
            build_id=self._build_id,
            project_name=self._project_name,
            architectures=self._architectures,
            timeout=timeout,
        )

    @property
    def build_id(self) -> str:
        """Get the build id."""
        return self._build_id

    def print_status(self) -> None:
        """Print the status of a remote build in Launchpad."""
        if self._lpc.has_outstanding_build():
            build_status = self._lpc.get_build_status()
            for arch, status in build_status.items():
                logger.info("Build status for arch %s: %s", arch, status)
        else:
            logger.info("No build task(s) found.")

    def has_outstanding_build(self) -> bool:
        """Check if there is an existing build on Launchpad.

        :returns: True if there is an existing (incomplete) build on Launchpad.
        """
        return self._lpc.has_outstanding_build()

    def monitor_build(self) -> None:
        """Monitor and periodically log the status of a remote build in Launchpad."""
        logger.info(
            "Building snap package for %s. This may take some time to finish.",
            humanize_list(self._lpc.architectures, "and", "{}"),
        )

        logger.info("Building...")
        try:
            self._lpc.monitor_build()
        finally:
            logger.info("Build task(s) complete.")

    def clean_build(self) -> None:
        """Clean the cache and Launchpad build."""
        logger.info("Cleaning existing builds and artefacts.")
        self._lpc.cleanup()
        self._worktree.clean_cache()

    def start_build(self) -> None:
        """Start a build in Launchpad.

        A local copy of the project is created and pushed to Launchpad via git.
        """
        self._worktree.init_repo()

        logger.debug("Cached project at %s", self._worktree.repo_dir)
        self._lpc.push_source_tree(repo_dir=self._worktree.repo_dir)

        self._lpc.start_build()
