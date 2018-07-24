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
import yaml

from testtools.matchers import Equals, FileExists

from tests import integration


class BuildPropertiesTestCase(integration.TestCase):
    def test_build(self):
        self.assert_expected_build_state("local-plugin-build-properties")

    def test_build_legacy_build_properties(self):
        self.assert_expected_build_state("local-plugin-legacy-build-properties")

    def assert_expected_build_state(self, project_dir):
        self.run_snapcraft("build", project_dir)

        state_file = os.path.join(self.parts_dir, "x-local-plugin", "state", "build")
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        # Verify that the correct schema dependencies made it into the state.
        self.assertTrue("foo" in state.schema_properties)
        self.assertTrue("stage-packages" in state.schema_properties)

        # Verify that the contents of the dependencies made it in as well.
        self.assertTrue("foo" in state.properties)
        self.assertTrue("stage-packages" in state.properties)
        self.assertThat(state.properties["foo"], Equals("bar"))
        self.assertThat(state.properties["stage-packages"], Equals(["curl"]))

    def test_build_with_arch(self):
        if self.deb_arch == "armhf":
            self.skipTest("For now, we just support crosscompile from amd64")
        self.run_snapcraft(["build", "--target-arch=i386", "go-hello"], "go-hello")
        state_file = os.path.join(self.parts_dir, "go-hello", "state", "build")
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)
        self.assertThat(state.project_options["deb_arch"], Equals("i386"))

    def test_arch_with_build(self):
        if self.deb_arch == "armhf":
            self.skipTest("For now, we just support crosscompile from amd64")
        self.run_snapcraft(["--target-arch=i386", "build", "go-hello"], "go-hello")
        state_file = os.path.join(self.parts_dir, "go-hello", "state", "build")
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)
        self.assertThat(state.project_options["deb_arch"], Equals("i386"))
