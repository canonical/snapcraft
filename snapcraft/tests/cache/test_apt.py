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

import os
from testtools.matchers import (
    DirExists,
    Not,
)

from snapcraft import tests
from snapcraft.internal import cache


class AptStagePackageCacheTestCase(tests.TestCase):

    def test_old_caches_get_cleaned(self):
        # The XDG dirs are mocked, so we can safely use AptStagePackageCache
        # and not mess with the real cache.
        package_cache = cache.AptStagePackageCache(sources_digest='1')

        # Assert that the cache was created.
        self.assertThat(os.path.join(
            package_cache.stage_package_cache_root, 'apt', '1'), DirExists())

        # Pretend that the sources list have changed, thus changing the digest.
        # This will also change the cache subdirectory used, which means the
        # old one should be cleaned up.
        package_cache = cache.AptStagePackageCache(sources_digest='2')

        self.assertThat(os.path.join(
            package_cache.stage_package_cache_root, 'apt', '1'),
            Not(DirExists()))
        self.assertThat(os.path.join(
            package_cache.stage_package_cache_root, 'apt', '2'), DirExists())
