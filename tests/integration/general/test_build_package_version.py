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

import apt
import testscenarios
from testtools.matchers import Equals

from tests import integration


class BuildPackageVersionTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = (
        ("global", dict(project="build-package", package="hello", part="hello")),
        (
            "local",
            dict(project="build-package-global", package="haskell-doc", part=None),
        ),
    )

    def test_build_package_gets_version(self):
        self.copy_project_to_cwd(self.project)
        version = self.set_build_package_version(
            os.path.join("snap", "snapcraft.yaml"), self.part, package=self.package
        )
        self.run_snapcraft("pull")

        with apt.Cache() as apt_cache:
            installed_version = apt_cache[self.package].candidate.version
        self.assertThat(installed_version, Equals(version))


class BuildPackageVersionErrorsTestCase(integration.TestCase):
    def test_build_package_with_invalid_version_must_fail(self):
        self.copy_project_to_cwd("build-package-global")
        self.set_build_package_version(
            os.path.join("snap", "snapcraft.yaml"),
            part=None,
            package="haskell-doc",
            version="invalid",
        )
        error = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, "pull"
        )
        self.assertIn(
            "Could not find a required package in 'build-packages': "
            "haskell-doc=invalid",
            str(error.output),
        )
