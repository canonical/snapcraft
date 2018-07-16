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

from testtools.matchers import (
    Annotate,
    Contains,
    DirExists,
    Equals,
    FileExists,
    MatchesRegex,
    Not,
)

from tests import integration, fixture_setup, skip


class RpathTestCase(integration.TestCase):
    def test_origin_rpath(self):
        self.run_snapcraft("prime", "rpath-test")

        self.assertThat(os.path.join(self.prime_dir, "binary"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "lib", "libfoo.so"), FileExists())

        # Assert that the $ORIGIN rpath did not result in the library being
        # pulled in twice.
        self.assertThat(
            os.path.join(self.prime_dir, os.path.abspath(self.prime_dir).lstrip("/")),
            Not(DirExists()),
        )


class Libc6TestCase(integration.TestCase):
    def test_primed_libc6(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "nil", "stage-packages": ["libc6", "hello"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")

        bin_path = os.path.join(self.prime_dir, "usr", "bin", "hello")
        self.assertThat(bin_path, FileExists())

        # Verify that the correct interpreter that has been staged is used.
        interpreter = subprocess.check_output(
            [self.patchelf_command, "--print-interpreter", bin_path]
        ).decode()
        expected_interpreter = r"^/snap/test-snap/current/lib/.*"
        self.assertThat(interpreter, MatchesRegex(expected_interpreter))


class OriginRPATHTestCase(integration.TestCase):
    def test_origin(self):
        # We stage libc6 for this to work on non xenial
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part", {"plugin": "nil", "stage-packages": ["libc6", "python3"]}
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")

        bin_path = os.path.join(self.prime_dir, "usr", "bin", "python3")
        self.assertThat(bin_path, FileExists())

        # Verify there aren't any duplicate rpath entries
        rpath = (
            subprocess.check_output([self.patchelf_command, "--print-rpath", bin_path])
            .decode()
            .strip()
        )
        seen = set()
        for i in rpath.split(":"):
            self.assertThat(
                seen, Annotate("A duplicate rpath was found!", Not(Contains(i)))
            )
            seen.add(i)

    @skip.skip_unless_codename("xenial", "we currently only have core")
    def test_origin_runpath_migrated_to_rpath(self):
        # We stage libc6 for this to work on non xenial
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, confinement="classic")
        snapcraft_yaml.update_part(
            "test-part",
            {
                "plugin": "nil",
                "build-packages": ["patchelf"],
                "stage-packages": ["hello"],
                # Without --force-rpath this is essentially a RUNPATH
                "override-build": (
                    "patchelf --set-rpath '$ORIGIN/foobar' "
                    "$SNAPCRAFT_PART_INSTALL/usr/bin/hello"
                ),
            },
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")

        bin_path = os.path.join(self.prime_dir, "usr", "bin", "hello")
        self.assertThat(bin_path, FileExists())

        # Verify there aren't any duplicate rpath entries
        readelf_d = (
            subprocess.check_output(["readelf", "-d", bin_path]).decode().strip()
        )
        lines_with_runpath = [l for l in readelf_d.splitlines() if "RUNPATH" in l]
        lines_with_rpath = [l for l in readelf_d.splitlines() if "RPATH" in l]
        self.assertThat(
            len(lines_with_runpath), Annotate("Expected no RUNPATH entries", Equals(0))
        )
        self.assertThat(
            len(lines_with_rpath), Annotate("Expected one RPATH entry", Equals(1))
        )


class ExecStackTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()
        if self.deb_arch == "arm64":
            self.skipTest("architecture is not supported by execstack")

    def _setup_project(self, keep_execstack: bool):
        if keep_execstack:
            attributes = ["keep-execstack"]
        else:
            attributes = []

        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part",
            {
                "plugin": "nil",
                "build-attributes": attributes,
                "build": (
                    "execstack --set-execstack $SNAPCRAFT_PART_INSTALL/usr/bin/hello"
                ),
                "prime": ["usr/bin/hello"],
                "build-packages": ["execstack"],
                "stage-packages": ["hello"],
            },
        )
        self.useFixture(snapcraft_yaml)

    def _assert_execstack(self, is_set: bool):
        stage_bin_path = os.path.join("stage", "usr", "bin", "hello")
        stage_bin_query = (
            subprocess.check_output([self.execstack_command, "--query", stage_bin_path])
            .decode()
            .strip()
        )
        # 'X' means the executable stack is required. See `man 8 execstack`.
        self.assertThat(stage_bin_query, Equals("X {}".format(stage_bin_path)))

        prime_bin_path = os.path.join("prime", "usr", "bin", "hello")
        prime_bin_query = (
            subprocess.check_output([self.execstack_command, "--query", prime_bin_path])
            .decode()
            .strip()
        )
        # '-' means the executable stack is not required.
        # See `man 8 execstack`.
        if is_set:
            expected_value = "X"
        else:
            expected_value = "-"
        self.assertThat(
            prime_bin_query, Equals("{} {}".format(expected_value, prime_bin_path))
        )

    def test_keep_execstack(self):
        self._setup_project(keep_execstack=True)
        self.run_snapcraft("prime")

        self._assert_execstack(is_set=True)

    def test_clear_execstack(self):
        self._setup_project(keep_execstack=False)
        self.run_snapcraft("prime")

        self._assert_execstack(is_set=False)
