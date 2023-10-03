# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019, 2023 Canonical Ltd
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

"""Manages tree for remote builds."""

from pathlib import Path

from snapcraft.remote.utils import rmtree


class WorkTree:
    """Class to manages tree for remote builds."""

    def __init__(self, worktree_dir: Path) -> None:
        """Create remote-build WorkTree.

        :param str worktree_dir: Directory to use for working tree.
        """
        # Working tree base directory.
        self._base_dir = worktree_dir

        # Initialize clean repo to ship to remote builder.
        self._repo_dir = self._base_dir / "repo"
        if self._repo_dir.exists():
            rmtree(self._repo_dir)
