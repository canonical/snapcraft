# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from snapcraft import project
from snapcraft.internal.project_loader import inspection

from testtools.matchers import Equals

from tests import unit


class ProvidesTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.part1 = self.load_part("part1")
        self.part2 = self.load_part("part2")
        self.parts = [self.part1, self.part2]
        self.project = project.Project()

        self.part1.makedirs()
        self.part2.makedirs()

    def test_provides_stage(self):
        file1_path = os.path.join(self.project.stage_dir, "file1")
        file2_path = os.path.join(self.project.stage_dir, "file2")

        self.part1.mark_stage_done({"file1"}, set())
        self.part2.mark_stage_done({"file2"}, set())

        for file_path in (file1_path, file2_path):
            open(file_path, "w").close()

        self.assertThat(
            inspection.provides(file1_path, self.project, self.parts),
            Equals({self.part1}),
        )
        self.assertThat(
            inspection.provides(file2_path, self.project, self.parts),
            Equals({self.part2}),
        )

    def test_provides_prime(self):
        file1_path = os.path.join(self.project.prime_dir, "file1")
        file2_path = os.path.join(self.project.prime_dir, "file2")

        self.part1.mark_prime_done({"file1"}, set(), set())
        self.part2.mark_prime_done({"file2"}, set(), set())

        open(file1_path, "w").close()
        open(file2_path, "w").close()

        self.assertThat(
            inspection.provides(file1_path, self.project, self.parts),
            Equals({self.part1}),
        )
        self.assertThat(
            inspection.provides(file2_path, self.project, self.parts),
            Equals({self.part2}),
        )

    def test_provides_dir(self):
        dir_path = os.path.join(self.project.stage_dir, "dir")
        file1_path = os.path.join(dir_path, "file1")
        file2_path = os.path.join(dir_path, "file2")

        self.part1.mark_stage_done({os.path.join("dir", "file1")}, {"dir"})
        self.part2.mark_stage_done({os.path.join("dir", "file2")}, {"dir"})

        for file_path in (file1_path, file2_path):
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            open(file_path, "w").close()

        self.assertThat(
            inspection.provides(dir_path, self.project, self.parts),
            Equals({self.part1, self.part2}),
        )

    def test_provides_file_outside_stage_or_prime(self):
        file_path = os.path.join(self.part1.plugin.sourcedir, "file")
        open(file_path, "w").close()

        raised = self.assertRaises(
            inspection.errors.ProvidesInvalidFilePathError,
            inspection.provides,
            file_path,
            self.project,
            self.parts,
        )

        self.assertThat(raised.path, Equals(file_path))

    def test_provides_untracked_file(self):
        file_path = os.path.join(self.project.stage_dir, "file")
        open(file_path, "w").close()

        raised = self.assertRaises(
            inspection.errors.UntrackedFileError,
            inspection.provides,
            file_path,
            self.project,
            self.parts,
        )

        self.assertThat(raised.path, Equals(file_path))

    def test_provides_no_such_file(self):
        file_path = os.path.join(self.project.stage_dir, "foo")

        raised = self.assertRaises(
            inspection.errors.NoSuchFileError,
            inspection.provides,
            file_path,
            self.project,
            self.parts,
        )

        self.assertThat(raised.path, Equals(file_path))
