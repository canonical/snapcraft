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
import testscenarios
from testtools.matchers import (
    Contains,
    FileContains,
    FileExists,
    MatchesRegex,
    Not,
    StartsWith,
)

from tests import integration, fixture_setup


class PrimeTestCase(integration.TestCase):
    def test_classic_confinement(self):
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        project_dir = "classic-build"

        # The first run should fail as the environment variable is not
        # set but we can only test this on clean systems.
        if not os.path.exists(os.path.join(os.path.sep, "snap", "core", "current")):
            try:
                self.run_snapcraft(["prime"], project_dir)
            except subprocess.CalledProcessError:
                pass
            else:
                self.fail("This should fail as SNAPCRAFT_SETUP_CORE is not set")

        # Now we set the required environment variable
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_SETUP_CORE", "1"))

        self.run_snapcraft(["prime"], project_dir)

        bin_path = os.path.join(self.prime_dir, "bin", "hello-classic")
        self.assertThat(bin_path, FileExists())

        interpreter = subprocess.check_output(
            [self.patchelf_command, "--print-interpreter", bin_path]
        ).decode()
        expected_interpreter = r"^/snap/core/current/.*"
        self.assertThat(interpreter, MatchesRegex(expected_interpreter))

        # We check stage to make sure the hard link is broken.
        staged_bin_path = os.path.join(self.stage_dir, "bin", "hello-classic")
        self.assertThat(staged_bin_path, FileExists())

        staged_interpreter = subprocess.check_output(
            [self.patchelf_command, "--print-interpreter", staged_bin_path]
        ).decode()
        self.assertThat(staged_interpreter, MatchesRegex(r"^/lib.*"))

    def test_classic_confinement_patchelf_disabled(self):
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        project_dir = "classic-build"

        # Now we set the required environment variable
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_SETUP_CORE", "1"))

        self.copy_project_to_cwd(project_dir)

        # Create a new snapcraft.yaml
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, confinement="classic")
        snapcraft_yaml.update_part(
            "hello",
            {"source": ".", "plugin": "make", "build-attributes": ["no-patchelf"]},
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("prime")

        bin_path = os.path.join(self.prime_dir, "bin", "hello-classic")
        self.assertThat(bin_path, FileExists())

        interpreter = subprocess.check_output(
            [self.patchelf_command, "--print-interpreter", bin_path]
        ).decode()
        self.assertThat(interpreter, StartsWith("/lib"))

    def test_classic_confinement_with_existing_rpath(self):
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        project_dir = "classic-build-existing-rpath"

        # The first run should fail as the environment variable is not
        # set but we can only test this on clean systems.
        if not os.path.exists(os.path.join(os.path.sep, "snap", "core", "current")):
            try:
                self.run_snapcraft(["prime"], project_dir)
            except subprocess.CalledProcessError:
                pass
            else:
                self.fail("This should fail as SNAPCRAFT_SETUP_CORE is not set")

        # Now we set the required environment variable
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_SETUP_CORE", "1"))

        self.run_snapcraft(["prime"], project_dir)

        bin_path = os.path.join(self.prime_dir, "bin", "hello-classic")
        self.assertThat(bin_path, FileExists())

        rpath = (
            subprocess.check_output([self.patchelf_command, "--print-rpath", bin_path])
            .decode()
            .strip()
        )
        expected_rpath = "$ORIGIN/../fake-lib:/snap/core/current/"
        self.assertThat(rpath, StartsWith(expected_rpath))

    def test_prime_includes_stage_fileset(self):
        self.run_snapcraft("prime", "prime-from-stage")
        self.assertThat(os.path.join(self.prime_dir, "without-a"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "without-b"), Not(FileExists()))
        self.assertThat(os.path.join(self.prime_dir, "without-c"), FileExists())

    def test_prime_includes_stage_excludes_fileset(self):
        self.run_snapcraft("prime", "prime-from-stage")
        self.assertThat(os.path.join(self.prime_dir, "with-a"), Not(FileExists()))
        self.assertThat(os.path.join(self.prime_dir, "with-b"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "with-c"), FileExists())

    def test_prime_with_non_ascii_desktop_file(self):
        # Originally, in this test we forced LC_ALL=C. However, now that we
        # are using the click python library we can't do it because it fails
        # to run any command when the system language is ascii.
        # --20170518 - elopio
        self.run_snapcraft("prime", "desktop-with-non-ascii")

        desktop_path = os.path.join(self.prime_dir, "meta", "gui", "test-app.desktop")

        self.expectThat(desktop_path, FileContains(matcher=Contains("non ascíí")))


class PrimedAssetsTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        ("setup", dict(project_dir="assets-with-gui-in-setup")),
        ("snap", dict(project_dir="assets-with-gui-in-snap")),
    ]

    def test_assets_in_meta(self):
        self.run_snapcraft("prime", self.project_dir)

        gui_dir = os.path.join(self.prime_dir, "meta", "gui")
        expected_desktop = dedent(
            """\
            [Desktop Entry]
            Name=My App
            Exec=my-app
            Type=Application
            """
        )
        self.expectThat(os.path.join(gui_dir, "icon.png"), FileExists())
        self.expectThat(
            os.path.join(gui_dir, "my-app.desktop"), FileContains(expected_desktop)
        )
