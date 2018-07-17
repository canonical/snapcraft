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
import subprocess

from testtools.matchers import Equals, Contains

from tests import integration


class AfterTestCase(integration.TestCase):
    def test_stage_dependencies(self):
        self.run_snapcraft("stage", "dependencies")

        self.assertTrue(os.access(os.path.join(self.stage_dir, "bin", "p3"), os.X_OK))

    def test_build_with_circular_dependencies(self):
        self.copy_project_to_cwd("dependencies")

        with open("snapcraft.yaml", "r") as snapcraft_yaml:
            snapcraft_yaml_contents = snapcraft_yaml.read()
        with open("snapcraft.yaml", "w") as snapcraft_yaml:
            snapcraft_yaml.write(
                snapcraft_yaml_contents.replace("p1:", "p1:\n    after: [p3]")
            )

        # We update here to get a clean log/stdout later
        self.run_snapcraft("update")

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, "build"
        )

        self.assertThat(exception.returncode, Equals(2))
        expected = (
            "Issue detected while analyzing snapcraft.yaml: "
            "circular dependency chain found in parts definition\n"
        )
        self.assertThat(exception.output, Contains(expected))

    def test_build_with_missing_dependencies(self):
        self.copy_project_to_cwd("dependencies")

        with open("snapcraft.yaml", "r") as snapcraft_yaml:
            snapcraft_yaml_contents = snapcraft_yaml.read()
        wrong_contents = snapcraft_yaml_contents.replace("    after: [p1]\n", "")
        wrong_contents = wrong_contents.replace("    after: [p2]\n", "")
        with open("snapcraft.yaml", "w") as snapcraft_yaml:
            snapcraft_yaml.write(wrong_contents)

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, "build"
        )

        self.assertThat(exception.returncode, Equals(2))

    def test_pull_with_tree_of_dependencies(self):
        self.run_snapcraft("pull", os.path.join("circular-dependencies", "tree"))

    def test_pull_with_circular_dependencies(self):
        self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft,
            "pull",
            os.path.join("circular-dependencies", "circle"),
        )
