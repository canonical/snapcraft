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
import subprocess
import tempfile
import yaml

from ._cache import SnapcraftProjectCache
from snapcraft import file_utils

logger = logging.getLogger(__name__)


class SnapCache(SnapcraftProjectCache):
    """Cache for snap revisions."""

    def __init__(self, *, project_name):
        super().__init__(project_name=project_name)
        self.snap_cache_root = self._setup_snap_cache_root()

    def _setup_snap_cache_root(self):
        snap_cache_root = os.path.join(self.project_cache_root, "snap_hashes")
        os.makedirs(snap_cache_root, exist_ok=True)
        return snap_cache_root

    def _get_snap_deb_arch(self, snap_filename):
        with tempfile.TemporaryDirectory() as temp_dir:
            unsquashfs_path = file_utils.get_tool_path("unsquashfs")
            output = subprocess.check_output(
                [
                    unsquashfs_path,
                    "-d",
                    os.path.join(temp_dir, "squashfs-root"),
                    snap_filename,
                    "-e",
                    os.path.join("meta", "snap.yaml"),
                ]
            )
            logger.debug(output)
            with open(
                os.path.join(temp_dir, "squashfs-root", "meta", "snap.yaml")
            ) as yaml_file:
                snap_yaml = yaml.safe_load(yaml_file)
        # XXX: add multiarch support later
        try:
            return snap_yaml["architectures"][0]
        except KeyError:
            return "all"

    def _get_snap_cache_path(self, snap_filename):
        snap_hash = file_utils.calculate_sha3_384(snap_filename)
        arch = self._get_snap_deb_arch(snap_filename)
        os.makedirs(os.path.join(self.snap_cache_root, arch), exist_ok=True)
        return os.path.join(self.snap_cache_root, arch, snap_hash)

    def cache(self, *, snap_filename):
        """Cache snap revision by sha3-384 hash in XDG cache, unless it already exists.
        :returns: path to cached revision.
        """
        cached_snap_path = self._get_snap_cache_path(snap_filename)
        try:
            if not os.path.isfile(cached_snap_path):
                # this must not be hard-linked, as rebuilding a snap
                # with changes should invalidate the cache, hence avoids
                # using fileutils.link_or_copy.
                shutil.copyfile(snap_filename, cached_snap_path)
        except OSError:
            logger.warning("Unable to cache snap {}.".format(snap_filename))
        return cached_snap_path

    def get(self, *, deb_arch, snap_hash=None):
        """Get the revision by sha3-384 hash or the latest cached item.

        :deb_arch: arch as string.
        :snap_hash: get by sha3 384 hash.

        :returns: full path to cached snap.
        """
        snap_cache_dir = os.path.join(self.snap_cache_root, deb_arch)
        if not os.path.isdir(snap_cache_dir):
            return None

        cached_hashes = os.listdir(snap_cache_dir)
        if not cached_hashes:
            return None

        if snap_hash:
            for cached_hash in cached_hashes:
                if cached_hash == snap_hash:
                    return os.path.join(snap_cache_dir, cached_hash)
            return None

        cached_snaps = [os.path.join(snap_cache_dir, f) for f in cached_hashes]
        return max(cached_snaps, key=os.path.getctime)

    def prune(self, *, deb_arch, keep_hash):
        """Prune the snap revisions beside the keep_hash in XDG cache.

        :returns: pruned files paths list.
        """

        pruned_files_list = []

        snap_cache_dir = os.path.join(self.snap_cache_root, deb_arch)
        for cached_hash in os.listdir(snap_cache_dir):
            if cached_hash != keep_hash:
                try:
                    cached_snap = os.path.join(snap_cache_dir, cached_hash)
                    os.remove(cached_snap)
                    pruned_files_list.append(cached_snap)
                except OSError:
                    logger.warning("Unable to prune snap {}.".format(cached_snap))
        return pruned_files_list
