# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
    FileExists
)

import integration_tests


class TarPluginTestCase(integration_tests.TestCase):

    def test_stage_zip_source(self):
        self.copy_project_to_cwd('simple-zip')
        self.run_snapcraft('stage')

        expected_files = [
            'top-simple',
            os.path.join('dir-simple', 'sub')
        ]
        for expected_file in expected_files:
            self.assertThat(
                os.path.join(self.stage_dir, expected_file),
                FileExists())
        expected_dirs = [
            'dir-simple',
        ]
        for expected_dir in expected_dirs:
            self.assertThat(
                os.path.join(self.stage_dir, expected_dir),
                DirExists())

        # Regression test for
        # https://bugs.launchpad.net/snapcraft/+bug/1500728
        self.run_snapcraft('pull')
