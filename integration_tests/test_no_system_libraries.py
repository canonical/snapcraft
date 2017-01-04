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

import glob
import os

from testtools.matchers import (
    DirExists,
    Not
)

import snapcraft

import integration_tests


class NoSystemLibrariesTestCase(integration_tests.TestCase):

    def test_system_libraries(self):
        project_dir = 'fake-curl-library'
        self.run_snapcraft(['prime', 'main-no-prereq'], project_dir)

        # Verify that the system's libcurl was pulled in.
        arch = snapcraft.ProjectOptions().arch_triplet
        libcurl_path = os.path.join(
            project_dir, 'prime', 'usr', 'lib', arch, 'libcurl.so*')
        self.assertTrue(glob.glob(libcurl_path + '*'))

    def test_no_system_libraries(self):
        project_dir = 'fake-curl-library'
        self.run_snapcraft(['prime', 'main-no-libs'], project_dir)

        # Verify that the system's libcurl was NOT pulled in.
        self.assertThat(
            os.path.join(project_dir, 'prime', 'usr'), Not(DirExists()))
