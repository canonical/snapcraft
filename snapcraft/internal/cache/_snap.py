# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import shutil

from ._cache import SnapcraftProjectCache
from snapcraft import file_utils

logger = logging.getLogger(__name__)


class SnapCache(SnapcraftProjectCache):
    """Cache for snap revisions."""
    def __init__(self, *, project_name):
        super().__init__(project_name=project_name)
        self.snap_cache_root = self._setup_snap_cache_root()

    def _setup_snap_cache_root(self):
        snap_cache_root = os.path.join(self.project_cache_root, 'snap_hashes')
        os.makedirs(snap_cache_root, exist_ok=True)
        return snap_cache_root

    def cache(self, snap_filename):
        """Cache snap revision by sha3-384 hash in XDG cache, unless it already exists.
        :returns: path to cached revision.
        """
        snap_hash = file_utils.calculate_sha3_384(snap_filename)
        cached_snap_path = os.path.join(self.snap_cache_root, snap_hash)
        try:
            if not os.path.isfile(cached_snap_path):
                # this must not be hard-linked, as rebuilding a snap
                # with changes should invalidate the cache, hence avoids
                # using fileutils.link_or_copy.
                shutil.copyfile(snap_filename, cached_snap_path)
        except OSError:
            logger.warning(
                'Unable to cache snap {}.'.format(snap_filename))
        return cached_snap_path

    def get(self, snap_hash=None):
        """Get the revision by sha3-384 hash or the latest cached item.

        :snap_hash: get by sha3 384 hash.

        :returns: full path to cached snap.
        """
        cached_hashes = os.listdir(self.snap_cache_root)
        if not cached_hashes:
            return None

        if snap_hash:
            for cached_hash in cached_hashes:
                if cached_hash == snap_hash:
                    return os.path.join(self.snap_cache_root, cached_hash)
            return None

        cached_snaps = [os.path.join(self.snap_cache_root, f)
                        for f in os.listdir(self.snap_cache_root)]
        return max(cached_snaps, key=os.path.getctime)

    def prune(self, *, keep_hash):
        """Prune the snap revisions beside the keep_hash in XDG cache.

        :returns: pruned files paths list.
        """

        pruned_files_list = []

        for snap_filename in os.listdir(self.snap_cache_root):
            cached_snap = os.path.join(self.snap_cache_root, snap_filename)
            if snap_filename != keep_hash:
                try:
                    os.remove(cached_snap)
                    pruned_files_list.append(cached_snap)
                except OSError:
                    logger.warning(
                        'Unable to prune snap {}.'.format(cached_snap))
        return pruned_files_list
