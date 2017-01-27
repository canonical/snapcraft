# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import fileinput
import os

from testtools.matchers import (
    FileExists,
    Not
)

from snapcraft import _options
import integration_tests


class HardlinkSrcOldTreeTestCase(integration_tests.TestCase):

    def test_build_old_tree_still_filters(self):
        self.copy_project_to_cwd('old-part-src')
        platform_architecture = _options._get_platform_architecture()
        arch = _options._ARCH_TRANSLATIONS[platform_architecture]['deb']
        with fileinput.FileInput(
                os.path.join(self.parts_dir, 'part-name', 'state', 'pull'),
                inplace=True) as pull_state:
            for line in pull_state:
                print(line.replace('$arch', arch), end='')

        self.run_snapcraft('build')

        # Assert that the file we wanted to snap made it into the builddir
        builddir = os.path.join(self.parts_dir, 'part-name', 'build')
        self.assertThat(os.path.join(builddir, 'foo'), FileExists())

        # Make sure that the build step still filters out the snapcraft-
        # specific data.
        self.assertThat(
            os.path.join(builddir, self.stage_dir), Not(FileExists()))
        self.assertThat(
            os.path.join(builddir, self.prime_dir),  Not(FileExists()))
        self.assertThat(os.path.join(builddir, 'snapcraft.yaml'),
                        Not(FileExists()))
