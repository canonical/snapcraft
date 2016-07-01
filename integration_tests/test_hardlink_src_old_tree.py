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

import os

from testtools.matchers import (
    FileExists,
    Not
)

import integration_tests


class HardlinkSrcOldTreeTestCase(integration_tests.TestCase):

    def test_build_old_tree_still_filters(self):
        project_dir = 'old-part-src'
        self.run_snapcraft('build', project_dir)

        # Assert that the file we wanted to snap made it into the builddir
        builddir = os.path.join(project_dir, 'parts', 'part-name', 'build')
        self.assertThat(os.path.join(builddir, 'foo'), FileExists())

        # Make sure that the build step still filters out the snapcraft-
        # specific data.
        self.assertThat(os.path.join(builddir, 'stage'), Not(FileExists()))
        self.assertThat(os.path.join(builddir, 'prime'),  Not(FileExists()))
        self.assertThat(os.path.join(builddir, 'snapcraft.yaml'),
                        Not(FileExists()))
