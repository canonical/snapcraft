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

"""Remote-build and related utilities."""

from .errors import (
    GitError,
    LaunchpadHttpsError,
    RemoteBuildError,
    RemoteBuildTimeoutError,
)
from .git import GitRepo, is_repo
from .launchpad import LaunchpadClient
from .utils import get_build_id, rmtree
from .worktree import WorkTree

__all__ = [
    "get_build_id",
    "is_repo",
    "rmtree",
    "GitError",
    "GitRepo",
    "LaunchpadClient",
    "LaunchpadHttpsError",
    "RemoteBuildError",
    "RemoteBuildTimeoutError",
    "WorkTree",
]
