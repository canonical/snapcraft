# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Marius Gripsgard (mariogrip@ubuntu.com)
# Copyright (C) 2017-2018 Canonical Ltd
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

import fixtures
import testscenarios
import yaml
from testtools.matchers import Equals, FileExists, MatchesRegex, Not

from tests import integration
from tests.matchers import HasArchitecture


class RustPluginBaseTestCase(integration.TestCase):
    def run_snapcraft(self, command, project_dir=None, debug=True):
        try:
            failed = True
            super().run_snapcraft(command, project_dir, debug)
            failed = False
        except subprocess.CalledProcessError:
            if self.deb_arch == "arm64":
                # https://github.com/rust-lang/rustup.sh/issues/82
                self.expectFailure(
                    "The rustup script does not support arm64.",
                    self.assertFalse,
                    failed,
                )
            else:
                raise


class RustPluginTestCase(RustPluginBaseTestCase):
    def test_stage_rust_plugin(self):
        self.run_snapcraft("stage", "rust-hello")

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, "bin", "rust-hello")
        )
        self.assertThat(binary_output, Equals("There is rust on snaps!\n"))

    def test_stage_rust_with_revision(self):
        self.run_snapcraft("stage", "rust-with-revision")

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, "bin", "rust-with-revision")
        )
        self.assertIn("Rust revision: 1.12.0", binary_output)

    def test_stage_rust_plugin_with_conditional_feature(self):
        self.run_snapcraft("stage", "rust-with-conditional")

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, "bin", "simple-rust")
        )
        self.assertThat(binary_output, Equals("Conditional features work!\n"))

    def test_stage_rust_with_source_subdir(self):
        self.run_snapcraft("stage", "rust-subdir")

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, "bin", "rust-subdir")
        )
        self.assertThat(binary_output, Equals("Rust in a subdirectory works\n"))
        # Test for bug https://bugs.launchpad.net/snapcraft/+bug/1654764
        self.assertThat("Cargo.lock", Not(FileExists()))

    def test_stage_rust_with_source_and_source_subdir(self):
        self.copy_project_to_cwd("rust-subdir")
        with open("snapcraft.yaml") as snapcraft_yaml_file:
            snapcraft_yaml = yaml.load(snapcraft_yaml_file)
        snapcraft_yaml["parts"]["rust-subdir"]["source"] = "."
        snapcraft_yaml["parts"]["rust-subdir"]["source-subdir"] = "subdir"
        with open("snapcraft.yaml", "w") as snapcraft_yaml_file:
            yaml.dump(snapcraft_yaml, snapcraft_yaml_file)

        self.run_snapcraft("pull")

        self.assertThat(
            os.path.join("parts", "rust-subdir", "src", "subdir", "Cargo.lock"),
            FileExists(),
        )

    def test_cross_compiling(self):
        if self.deb_arch != "amd64":
            self.skipTest("The test only handles amd64 to arm64")

        self.run_snapcraft(["build", "--target-arch=arm64"], "rust-hello")
        binary = os.path.join(
            self.parts_dir, "rust-hello", "install", "bin", "rust-hello"
        )
        self.assertThat(binary, HasArchitecture("aarch64"))


class RustPluginConfinementTestCase(
    testscenarios.WithScenarios, RustPluginBaseTestCase
):

    scenarios = (
        ("classic", dict(confinement="classic", startswith="/snap/")),
        ("strict", dict(confinement="strict", startswith="/lib")),
    )

    def _set_confinement(self, snapcraft_yaml_file):
        with open(snapcraft_yaml_file) as f:
            snapcraft_yaml = yaml.load(f)
        snapcraft_yaml["confinement"] = self.confinement
        with open(snapcraft_yaml_file, "w") as f:
            yaml.dump(snapcraft_yaml, f)

    def test_prime(self):
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_SETUP_CORE", "1"))
        self.copy_project_to_cwd("rust-hello")
        self._set_confinement("snapcraft.yaml")

        self.run_snapcraft("prime")

        bin_path = os.path.join("prime", "bin", "rust-hello")

        interpreter = subprocess.check_output(
            [self.patchelf_command, "--print-interpreter", bin_path]
        ).decode()
        expected_interpreter = r"^{}.*".format(self.startswith)
        self.assertThat(interpreter, MatchesRegex(expected_interpreter))
