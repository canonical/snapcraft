# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import logging
import os

from ._cache import SnapcraftStagePackageCache

logger = logging.getLogger(__name__)


class AptStagePackageCache(SnapcraftStagePackageCache):
    """Cache for stage-packages coming from apt."""

    def __init__(self, *, sources_digest):
        """Create a new AptStagePackageCache.

        :param str sources_digest: Unique digest of the current apt sources.
        """

        super().__init__()
        cache_base_dir = os.path.join(self.stage_package_cache_root, "apt")

        # TODO: Clean old cache, LP: #1663051

        self.base_dir = os.path.join(cache_base_dir, sources_digest)
        self.packages_dir = os.path.join(
            self.base_dir, "var", "cache", "apt", "archives"
        )
        os.makedirs(self.packages_dir, exist_ok=True)
