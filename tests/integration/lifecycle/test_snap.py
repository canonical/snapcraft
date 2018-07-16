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
from textwrap import dedent

import fixtures
import testtools
from testtools.matchers import Equals, EndsWith, FileContains, FileExists, Not

from tests import integration, skip


class SnapTestCase(integration.TestCase):
    @skip.skip_unless_codename("xenial", "test designed for xenial")
    def test_snap(self):
        self.copy_project_to_cwd("assemble")
        self.run_snapcraft("snap")

        snap_file_path = "assemble_1.0_{}.snap".format(self.deb_arch)
        self.assertThat(snap_file_path, FileExists())

        binary1_wrapper_path = os.path.join(
            self.prime_dir, "command-assemble-bin.wrapper"
        )
        with open("binary1.after", "r") as file_:
            expected_binary1_wrapper = file_.read()
        self.assertThat(binary1_wrapper_path, FileContains(expected_binary1_wrapper))

        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAP", os.path.join(os.getcwd(), self.prime_dir)
            )
        )
        binary_scenarios = (
            ("command-assemble-service.wrapper", "service-start\n"),
            ("stop-command-assemble-service.wrapper", "service-stop\n"),
            ("command-assemble-bin.wrapper", "binary1\n"),
            ("command-binary2.wrapper", "binary2\n"),
        )
        for binary, expected_output in binary_scenarios:
            output = subprocess.check_output(
                os.path.join(self.prime_dir, binary), universal_newlines=True
            )
            self.assertThat(output, Equals(expected_output))

        with testtools.ExpectedException(subprocess.CalledProcessError):
            subprocess.check_output(
                os.path.join(self.prime_dir, "bin", "not-wrapped"),
                stderr=subprocess.STDOUT,
            )

        self.assertThat(
            os.path.join(self.prime_dir, "bin", "not-wrapped.wrapper"),
            Not(FileExists()),
        )

        self.assertThat(
            os.path.join(
                self.prime_dir, "bin", "command-binary-wrapper-none.wrapper.wrapper"
            ),
            Not(FileExists()),
        )

        # LP: #1750658
        self.assertThat(
            os.path.join(self.prime_dir, "meta", "snap.yaml"),
            FileContains(
                dedent(
                    """\
                name: assemble
                version: 1.0
                summary: one line summary
                description: a longer description
                architectures:
                - {}
                confinement: strict
                grade: stable
                apps:
                  assemble-bin:
                    command: command-assemble-bin.wrapper
                  assemble-service:
                    command: command-assemble-service.wrapper
                    daemon: simple
                    stop-command: stop-command-assemble-service.wrapper
                  binary-wrapper-none:
                    command: subdir/binary3
                  binary2:
                    command: command-binary2.wrapper
            """
                ).format(self.deb_arch)
            ),
        )

    def test_snap_default(self):
        self.copy_project_to_cwd("assemble")
        self.run_snapcraft([])
        snap_file_path = "assemble_1.0_{}.snap".format(self.deb_arch)
        self.assertThat(snap_file_path, FileExists())

    def test_snap_directory(self):
        self.copy_project_to_cwd("assemble")
        self.run_snapcraft("snap")

        snap_file_path = "assemble_1.0_{}.snap".format(self.deb_arch)
        os.remove(snap_file_path)

        # Verify that Snapcraft can snap its own snap directory (this will make
        # sure `snapcraft snap` and `snapcraft snap <directory>` are always in
        # sync).
        self.run_snapcraft(["snap", "prime"])
        self.assertThat(snap_file_path, FileExists())

    def test_pack_directory(self):
        self.copy_project_to_cwd("assemble")
        self.run_snapcraft("snap")

        snap_file_path = "assemble_1.0_{}.snap".format(self.deb_arch)
        os.remove(snap_file_path)

        # Verify that Snapcraft can snap its own snap directory (this will make
        # sure `snapcraft snap` and `snapcraft pack <directory>` are always
        # in sync).
        self.run_snapcraft(["pack", "prime"])
        self.assertThat(snap_file_path, FileExists())

    def test_snap_long_output_option(self):
        self.run_snapcraft(["snap", "--output", "mysnap.snap"], "assemble")
        self.assertThat("mysnap.snap", FileExists())

    def test_snap_short_output_option(self):
        self.run_snapcraft(["snap", "-o", "mysnap.snap"], "assemble")
        self.assertThat("mysnap.snap", FileExists())

    def test_error_with_unexistent_build_package(self):
        self.copy_project_to_cwd("assemble")
        with open("snapcraft.yaml", "a") as yaml_file:
            yaml_file.write("build-packages:\n  - inexistent-package\n")

        # We update here to get a clean log/stdout later
        self.run_snapcraft("update")

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, "snap"
        )
        expected = (
            "Could not find a required package in 'build-packages': "
            "inexistent-package\n"
        )
        self.assertThat(exception.output, EndsWith(expected))

    @skip.skip_unless_codename("xenial", "test designed for xenial")
    def test_snap_with_exposed_files(self):
        self.copy_project_to_cwd("nil-plugin-pkgfilter")
        self.run_snapcraft("stage")
        self.assertThat(
            os.path.join(self.stage_dir, "usr", "bin", "nmcli"), FileExists()
        )

        self.run_snapcraft("snap")
        self.assertThat(
            os.path.join(self.prime_dir, "usr", "bin", "nmcli"), FileExists()
        )
        self.assertThat(
            os.path.join(self.prime_dir, "usr", "bin", "nmtui"), Not(FileExists())
        )

    def test_snap_from_snapcraft_init(self):
        self.assertThat("snapcraft.yaml", Not(FileExists()))
        self.run_snapcraft("init")
        self.assertThat(os.path.join("snap", "snapcraft.yaml"), FileExists())

        self.run_snapcraft("snap")

    def test_snap_with_arch(self):
        if self.deb_arch == "armhf":
            self.skipTest("For now, we just support crosscompile from amd64")
        self.run_snapcraft("init")

        self.run_snapcraft(["snap", "--target-arch=i386"])
        self.assertThat("my-snap-name_0.1_i386.snap", FileExists())

    def test_arch_with_snap(self):
        if self.deb_arch == "armhf":
            self.skipTest("For now, we just support crosscompile from amd64")
        self.run_snapcraft("init")

        self.run_snapcraft(["--target-arch=i386", "snap"])
        self.assertThat("my-snap-name_0.1_i386.snap", FileExists())

    def test_implicit_command_with_arch(self):
        if self.deb_arch == "armhf":
            self.skipTest("For now, we just support crosscompile from amd64")
        self.run_snapcraft("init")

        self.run_snapcraft("--target-arch=i386")
        self.assertThat("my-snap-name_0.1_i386.snap", FileExists())

    def test_error_on_bad_yaml(self):
        error = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, "stage", "bad-yaml"
        )
        self.assertIn(
            "Issues while validating snapcraft.yaml: found character '\\t' "
            "that cannot start any token on line 13 of snapcraft.yaml",
            str(error.output),
        )

    def test_yaml_merge_tag(self):
        self.copy_project_to_cwd("yaml-merge-tag")
        self.run_snapcraft("stage")
        self.assertThat(os.path.join(self.stage_dir, "test.txt"), FileExists())

    def test_ordered_snap_yaml(self):
        with open("snapcraft.yaml", "w") as s:
            s.write(
                dedent(
                    """\
                apps:
                    stub-app:
                        command: sh
                grade: stable
                version: "2"
                assumes: [snapd_227]
                architectures: [all]
                description: stub description
                summary: stub summary
                confinement: strict
                name: stub-snap
                environment:
                    stub_key: stub-value
                epoch: 1
                parts:
                    nothing:
                        plugin: nil
            """
                )
            )
        self.run_snapcraft("prime")

        expected_snap_yaml = dedent(
            """\
            name: stub-snap
            version: '2'
            summary: stub summary
            description: stub description
            architectures:
            - all
            confinement: strict
            grade: stable
            assumes:
            - snapd_227
            epoch: 1
            environment:
              stub_key: stub-value
            apps:
              stub-app:
                command: command-stub-app.wrapper
        """
        )
        self.assertThat(
            os.path.join("prime", "meta", "snap.yaml"), FileContains(expected_snap_yaml)
        )
