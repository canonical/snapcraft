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

import os

from testtools.matchers import DirExists, FileContains, Not

from tests import integration


class FilesetsTestCase(integration.TestCase):
    def test_filesets(self):
        self.run_snapcraft("snap", "make-with-filesets")

        expected_dirs = (
            os.path.join(self.stage_dir, "share"),
            os.path.join(self.stage_dir, "bin"),
            os.path.join(self.prime_dir, "bin"),
        )
        for expected_dir in expected_dirs:
            self.assertThat(expected_dir, DirExists())

        self.assertThat(os.path.join(self.prime_dir, "share"), Not(DirExists()))

        expected_files = (
            (os.path.join(self.stage_dir, "share", "share1"), "share1\n"),
            (os.path.join(self.stage_dir, "share", "share2"), "share2\n"),
            (os.path.join(self.stage_dir, "bin", "bin1"), "bin1\n"),
            (os.path.join(self.stage_dir, "bin", "bin2"), "bin2\n"),
            (os.path.join(self.prime_dir, "bin", "bin1"), "bin1\n"),
            (os.path.join(self.prime_dir, "bin", "bin2"), "bin2\n"),
        )
        for path, content in expected_files:
            self.assertThat(path, FileContains(content))
