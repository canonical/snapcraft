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

import subprocess

import apt

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
