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
import subprocess

from testtools.matchers import Equals, FileContains

from tests import integration


class EnvironmentTestCase(integration.TestCase):
    def test_environment(self):
        self.run_snapcraft("stage", "snapcraft-environment")

        part_install_dir = os.path.join(self.path, self.parts_dir, "env", "install")

        test_name = os.path.join(self.stage_dir, "test_name")
        self.assertThat(test_name, FileContains("test-environment"))

        test_version = os.path.join(self.stage_dir, "test_version")
        self.assertThat(test_version, FileContains("0.1"))

        test_stage = os.path.join(self.stage_dir, "test_stage")
        self.assertThat(
            test_stage, FileContains(os.path.join(self.path, self.stage_dir))
        )

        test_part_install = os.path.join(self.path, self.stage_dir, "test_part_install")
        self.assertThat(test_part_install, FileContains(part_install_dir))

    def test_project_environment_within_snapcraft(self):
        """Replace the SNAPCRAFT_PROJECT_.* occurrences in snapcraft.yaml

        If the correct environment SNAPCRAFT_PROJECT values aren't replaced
        this test will fail its pull step."""
        self.run_snapcraft("pull", "snapcraft-key-values")

    def test_snapcraft_stage_env_replacement(self):
        self.run_snapcraft("stage", "stage_env")

    def test_stage_cmake_plugin_with_replace(self):
        """Replace SNAPCRAFT_PART_* in the part's attributes"""
        self.run_snapcraft("stage", "cmake-with-env-var")

        binary_output = subprocess.check_output(
            [os.path.join(self.stage_dir, "bin", "cmake-with-env-var")]
        )
        sourcedir = os.path.join(self.path, self.parts_dir, "cmake-project", "src")
        builddir = os.path.join(self.path, self.parts_dir, "cmake-project", "build")
        installdir = os.path.join(self.path, self.parts_dir, "cmake-project", "install")
        self.assertThat(
            binary_output.decode("utf-8"),
            Equals(
                "I was built with:\n"
                "PART_SRC: {}\n"
                "PART_BUILD: {}\n"
                "PART_INSTALL: {}\n".format(sourcedir, builddir, installdir)
            ),
        )
