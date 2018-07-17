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

import os

from testtools.matchers import FileExists, Equals

from tests import integration


class YamlInSubdirTestCase(integration.TestCase):
    def test_stage(self):
        self.copy_project_to_cwd("yaml-in-subdir")
        os.chdir("subdir")
        self.run_snapcraft("stage")

        expected_file = os.path.join(self.stage_dir, "file")
        self.assertThat(expected_file, FileExists())
        with open(expected_file) as f:
            self.assertThat(f.read(), Equals("I'm a file\n"))

        expected_file = os.path.join(self.stage_dir, "subdirfile")
        self.assertThat(expected_file, FileExists())
        with open(expected_file) as f:
            self.assertThat(f.read(), Equals("I'm in the subdir\n"))
