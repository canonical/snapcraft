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
import textwrap

from snapcraft.internal import steps
from snapcraft.internal.project_loader import inspection

from testtools.matchers import Equals

from .. import ProjectLoaderBaseTest


class LifecycleStatusTest(ProjectLoaderBaseTest):
    def setUp(self):
        super().setUp()

        for path in ("src1", "src2"):
            os.mkdir(path)

        self.config = self.make_snapcraft_project(
            textwrap.dedent(
                """\
                name: my-snap-name
                version: '1.0'
                summary: summary
                description: description

                grade: devel
                confinement: devmode

                parts:
                    part1:
                        plugin: nil
                        source: src1/
                    part2:
                        plugin: nil
                        source: src2/
                        after: [part1]
                """
            )
        )

        for part in self.config.all_parts:
            part.makedirs()

        self.part1 = self.config.parts.get_part("part1")
        self.part2 = self.config.parts.get_part("part2")

    def test_lifecycle_status(self):
        # Stage part1, since it's a dependency
        self.part1.mark_pull_done()
        self.part1.mark_build_done()
        self.part1.mark_stage_done(set(), set())

        # Pull part2
        self.part2.mark_pull_done()

        self.assertThat(
            inspection.lifecycle_status(self.config),
            Equals(
                [
                    {
                        "part": "part1",
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": None,
                    },
                    {
                        "part": "part2",
                        "pull": "complete",
                        "build": None,
                        "stage": None,
                        "prime": None,
                    },
                ]
            ),
        )

        # Now prime them both
        self.part1.mark_prime_done(set(), set(), set())
        self.part2.mark_build_done()
        self.part2.mark_stage_done(set(), set())
        self.part2.mark_prime_done(set(), set(), set())

        self.assertThat(
            inspection.lifecycle_status(self.config),
            Equals(
                [
                    {
                        "part": "part1",
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                    {
                        "part": "part2",
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                ]
            ),
        )

        # Change the source of part2, which should make its pull step outdated
        open(os.path.join("src2", "file"), "w").close()
        self.assertThat(
            inspection.lifecycle_status(self.config),
            Equals(
                [
                    {
                        "part": "part1",
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                    {
                        "part": "part2",
                        "pull": "outdated (source changed)",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "complete",
                    },
                ]
            ),
        )

        # Now clean the prime step of part1 and verify that it effects part2
        self.part1.mark_cleaned(steps.PRIME)
        self.assertThat(
            inspection.lifecycle_status(self.config),
            Equals(
                [
                    {
                        "part": "part1",
                        "pull": "complete",
                        "build": "complete",
                        "stage": "complete",
                        "prime": None,
                    },
                    {
                        "part": "part2",
                        "pull": "outdated (source changed)",
                        "build": "complete",
                        "stage": "complete",
                        "prime": "dirty ('part1' changed)",
                    },
                ]
            ),
        )
