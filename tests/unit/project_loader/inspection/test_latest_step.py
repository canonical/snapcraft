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

from snapcraft import project
from snapcraft.internal import steps
from snapcraft.internal.project_loader import inspection

from testtools.matchers import Equals
from unittest import mock

from tests import unit


class LatestStepTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.part1 = self.load_part("part1")
        self.part2 = self.load_part("part2")
        self.parts = [self.part1, self.part2]
        self.project = project.Project()

        self.part1.makedirs()
        self.part2.makedirs()

    def test_latest_step(self):
        # Pretend part2 has been pulled, and then pretend part1 has been built
        self.part2.mark_pull_done()
        self.part1.mark_build_done()

        self.assertThat(
            inspection.latest_step(self.parts),
            Equals((self.part1, steps.BUILD, mock.ANY)),
        )

        # Now pretend part2 was built, and verify that it's now the latest step
        self.part2.mark_build_done()
        self.assertThat(
            inspection.latest_step(self.parts),
            Equals((self.part2, steps.BUILD, mock.ANY)),
        )

    def test_latest_step_no_steps_run(self):
        self.assertRaises(
            inspection.errors.NoStepsRunError, inspection.latest_step, self.parts
        )
