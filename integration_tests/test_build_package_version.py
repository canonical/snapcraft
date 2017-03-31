# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import fixtures

import integration_tests


class BuildPackageVersionTestCase(integration_tests.TestCase):

    def test_build_package_gets_version(self):
        self.run_snapcraft('pull', 'build-package-version')
        pkg = 'hello'
        expected_version = '2.10-1'
        with apt.Cache() as apt_cache:
            installed_version = apt_cache[pkg].candidate.version
            self.assertEqual(expected_version,
                             installed_version)

    def test_global_build_package_gets_version(self):
        self.run_snapcraft('pull', 'build-package-version-global')
        pkg = 'hello'
        expected_version = '2.10-1'
        with apt.Cache() as apt_cache:
            installed_version = apt_cache[pkg].candidate.version
            self.assertEqual(expected_version,
                             installed_version)

    def test_build_package_bad_version(self):
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'pull', 'build-package-version-bad')
        self.assertIn(
            "Version 'x.y-z' for 'hello' was not found",
            str(error.output)
        )

    def test_build_packages_annotated_in_snapcraft_yaml(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))
        self.run_snapcraft('snap', 'build-package-version')
        snapcraft_yaml_path = os.path.join('prime', 'snap', 'snapcraft.yaml')

        self.assertTrue(os.path.exists(snapcraft_yaml_path))
        with open(snapcraft_yaml_path) as fp:
            self.assertIn("build-packages: [hello=2.10-1]",
                          fp.read())

    def test_build_packages_by_default_no_snapcraft_yaml(self):
        self.run_snapcraft('snap', 'build-package-version')
        snapcraft_yaml_path = os.path.join('prime', 'snap', 'snapcraft.yaml')

        self.assertTrue(not os.path.exists(snapcraft_yaml_path))
