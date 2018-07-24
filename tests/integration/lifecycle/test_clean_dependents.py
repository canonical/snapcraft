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

from testtools.matchers import Contains, DirExists, Not

from tests import integration


class CleanDependentsTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()

        self.copy_project_to_cwd("dependencies")
        self.run_snapcraft("prime")

        # Need to use the state directory here instead of partdir due to
        # bug #1567054.
        self.state_dirs = {
            "p1": os.path.join(self.parts_dir, "p1", "state"),
            "p2": os.path.join(self.parts_dir, "p2", "state"),
            "p3": os.path.join(self.parts_dir, "p3", "state"),
            "p4": os.path.join(self.parts_dir, "p4", "state"),
        }

    def assert_clean(self, parts, common=False):
        for part in parts:
            self.expectThat(
                self.state_dirs[part],
                Not(DirExists()),
                "Expected state directory for {!r} to be cleaned".format(part),
            )

        if common:
            self.expectThat(
                self.parts_dir,
                Not(DirExists()),
                "Expected parts/ directory to be cleaned",
            )
            self.expectThat(
                self.stage_dir,
                Not(DirExists()),
                "Expected stage/ directory to be cleaned",
            )
            self.expectThat(
                self.prime_dir,
                Not(DirExists()),
                "Expected prime/ directory to be cleaned",
            )

    def assert_not_clean(self, parts, common=False):
        for part in parts:
            self.expectThat(
                self.state_dirs[part],
                DirExists(),
                "Expected state directory for {!r} to not be cleaned".format(part),
            )

        if common:
            self.expectThat(
                self.parts_dir,
                DirExists(),
                "Expected parts/ directory to not be cleaned",
            )
            self.expectThat(
                self.stage_dir,
                DirExists(),
                "Expected stage/ directory to not be cleaned",
            )
            self.expectThat(
                self.prime_dir,
                DirExists(),
                "Expected prime/ directory to not be cleaned",
            )

    def test_clean_nested_dependent(self):
        # Test that p3 (which has dependencies but no dependents) cleans with
        # no extra parameters.
        self.run_snapcraft(["clean", "p3"])
        self.assert_clean(["p3"])
        self.assert_not_clean(["p1", "p2", "p4"], True)

        # Now run prime again
        self.run_snapcraft("prime")
        self.assert_not_clean(["p1", "p2", "p3", "p4"], True)

    def test_clean_dependent(self):
        # Test that p2 (which has both dependencies and dependents) cleans with
        # its dependents (p3 and p4) also specified.
        self.run_snapcraft(["clean", "p2", "p3", "p4"])
        self.assert_clean(["p2", "p3", "p4"])
        self.assert_not_clean(["p1"], True)

        # Now run prime again
        self.run_snapcraft("prime")
        self.assert_not_clean(["p1", "p2", "p3", "p4"], True)

    def test_clean_main(self):
        # Test that p1 (which has no dependencies but dependents) cleans with
        # its dependents (p2 and, as an extension, p3 and p4) also specified.
        self.run_snapcraft(["clean", "p1", "p2", "p3", "p4"])
        self.assert_clean(["p1", "p2", "p3", "p4"], True)

        # Now run prime again
        self.run_snapcraft("prime")
        self.assert_not_clean(["p1", "p2", "p3", "p4"], True)

    def test_clean_dependent_without_nested_dependents(self):
        # Test that cleaning p2 marks dependents as dirty, but doesn't clean
        # them
        output = self.run_snapcraft(["clean", "p2"])
        self.assertThat(
            output,
            Contains(
                "Cleaned 'p2', which makes the following parts out of date: 'p3' "
                "and 'p4'"
            ),
        )
        self.assert_not_clean(["p1", "p3", "p4"])
        self.assert_clean(["p2"])

    def test_clean_dependent_without_nested_dependent(self):
        # Test that cleaning p2 marks dependents as dirty, but doesn't clean
        # them if they're not specified
        output = self.run_snapcraft(["clean", "p2", "p3"])
        self.assertThat(
            output,
            Contains("Cleaned 'p2', which makes the following part out of date: 'p4'"),
        )
        self.assert_not_clean(["p1", "p4"])
        self.assert_clean(["p2", "p3"])

    def test_clean_main_without_any_dependent(self):
        # Test that p1 (which has no dependencies but dependents)
        # can be cleaned, but marks everything else as dirty
        output = self.run_snapcraft(["clean", "p1"])
        self.assertThat(
            output,
            Contains(
                "Cleaned 'p1', which makes the following parts out of date: 'p2', "
                "'p3', and 'p4'"
            ),
        )
        self.assert_clean(["p1"])
        self.assert_not_clean(["p2", "p3", "p4"])

    def test_clean_main_without_dependent(self):
        # Test that p1 and its nested dependencies can be cleaned while p2 is
        # marked as dirty
        output = self.run_snapcraft(["clean", "p1", "p3", "p4"])
        self.assertThat(
            output,
            Contains("Cleaned 'p1', which makes the following part out of date: 'p2'"),
        )
        self.assert_clean(["p1", "p3", "p4"])
        self.assert_not_clean(["p2"])

    def test_clean_main_without_nested_dependent(self):
        # Test that p1 and p2 clean and mark the nested dependents as dirty,
        # but don't clean them
        output = self.run_snapcraft(["clean", "p1", "p2"])
        self.assertThat(
            output,
            Contains(
                "Cleaned 'p2', which makes the following parts out of date: 'p3' "
                "and 'p4'"
            ),
        )
        self.assert_clean(["p1", "p2"])
        self.assert_not_clean(["p3", "p4"])
