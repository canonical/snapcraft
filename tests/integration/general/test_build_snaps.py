# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
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

import testscenarios
from testtools.matchers import Equals, Contains

from tests import fixture_setup, integration
from tests.integration import repo


class BuildSnapsTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = (
        ("snap name", {"snap": "u1test-snap-with-tracks"}),
        (
            "snap name with track and risk",
            {"snap": "u1test-snap-with-tracks/test-track-1/beta"},
        ),
    )

    def test_build_snap(self):
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        self.useFixture(fixture_setup.WithoutSnapInstalled(self.snap))
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part-with-build-snap", {"plugin": "nil", "build-snaps": [self.snap]}
        )
        self.useFixture(snapcraft_yaml)
        self.run_snapcraft("build")
        self.assertTrue(repo.is_snap_installed(self.snap))


class BuildSnapsErrorsTestCase(integration.TestCase):
    def test_inexistent_build_snap(self):
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part-with-build-snap",
            {"plugin": "nil", "build-snaps": ["inexistent"]},
        )
        self.useFixture(snapcraft_yaml)

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ["build"]
        )

        self.assertThat(exception.returncode, Equals(2))
        self.assertThat(exception.output, Contains("'inexistent'"))
        self.assertFalse(repo.is_snap_installed("inexistent"))

    def test_snap_exists_but_not_on_channel(self):
        # If the snap tested here does not exist, then BuildSnapsTestCase
        # will fail.
        if os.environ.get("ADT_TEST") and self.deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part-with-build-snap",
            {
                "plugin": "nil",
                "build-snaps": ["u1test-snap-with-tracks/not-exists/candidate"],
            },
        )
        self.useFixture(snapcraft_yaml)

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ["build"]
        )

        self.assertThat(exception.returncode, Equals(2))
        self.assertThat(exception.output, Contains("'u1test-snap-with-tracks'"))

        self.assertFalse(repo.is_snap_installed("u1test-snap-with-tracks"))
