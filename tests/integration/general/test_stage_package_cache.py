# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import glob
import os
import xdg

from testtools.matchers import Equals, HasLength

from tests import integration


class StagePackageCacheTestCase(integration.TestCase):
    def test_stage_package_gets_cached(self):
        self.run_snapcraft(["pull", "oneflatwithstagepackages"], "dump")

        # Verify the 'hello' deb package was cached.
        cache_dir = os.path.join(
            xdg.BaseDirectory.xdg_cache_home, "snapcraft", "stage-packages", "apt"
        )
        archive_dir = os.path.join("var", "cache", "apt", "archives")
        cached = glob.glob(os.path.join(cache_dir, "*", archive_dir, "hello*"))
        self.assertThat(cached, HasLength(1))
        cached_deb = cached[0]

        staged = glob.glob(
            os.path.join(
                "parts", "oneflatwithstagepackages", "ubuntu", "download", "hello*"
            )
        )
        self.assertThat(staged, HasLength(1))
        staged_deb = staged[0]

        # Verify that the staged and cached debs are the same file (hard
        # linked) by comparing inodes.
        cached_deb_inode = os.stat(cached_deb).st_ino
        self.assertThat(cached_deb_inode, Equals(os.stat(staged_deb).st_ino))

        # Now clean the part and pull again.
        self.run_snapcraft("clean", "dump")
        self.run_snapcraft(["pull", "oneflatwithstagepackages"], "dump")

        # Verify that the staged deb is _still_ the same one from the cache.
        self.assertThat(cached_deb_inode, Equals(os.stat(staged_deb).st_ino))
