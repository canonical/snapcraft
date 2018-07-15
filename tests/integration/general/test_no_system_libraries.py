# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from testtools.matchers import DirExists, Not

from tests import integration


class NoSystemLibrariesTestCase(integration.TestCase):
    def test_system_libraries(self):
        self.run_snapcraft(["prime", "main-no-prereq"], "fake-curl-library")

        # Verify that the system's libcurl was pulled in.
        libcurl_path = os.path.join(
            self.prime_dir, "usr", "lib", self.arch_triplet, "libcurl.so*"
        )
        self.assertTrue(glob.glob(libcurl_path + "*"))

    def test_no_system_libraries(self):
        self.run_snapcraft(["prime", "main-no-libs"], "fake-curl-library")

        # Verify that the system's libcurl was NOT pulled in.
        self.assertThat(os.path.join(self.prime_dir, "usr"), Not(DirExists()))
