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
import testscenarios
import yaml
from testtools.matchers import Equals

import integration_tests


class BuildPackageVersionTestCase(testscenarios.WithScenarios,
                                  integration_tests.TestCase):

    scenarios = (
        ('global', dict(project='build-package-version')),
        ('local', dict(project='build-package-version-global')),
    )

    def setUp(self):
        super().setUp()
        self.pkg_name = 'hello'
        self.pkg_version = integration_tests.get_package_version(
            self.pkg_name, self.distro_series, self.deb_arch)
        self.hello_package = '{}={}'.format(self.pkg_name, self.pkg_version)

    def _set_hello_package_version(self, snapcraft_yaml_file):
        with open(snapcraft_yaml_file) as f:
            snapcraft_yaml = yaml.load(f)
        if 'build-packages' in snapcraft_yaml:
            snapcraft_yaml['build-packages'] = [self.hello_package]
        else:
            snapcraft_yaml['parts']['hello']['build-packages'] = \
                [self.hello_package]
        with open(snapcraft_yaml_file, 'w') as f:
            yaml.dump(snapcraft_yaml, f)

    def test_build_package_gets_version(self):
        self.copy_project_to_cwd(self.project)
        self._set_hello_package_version(os.path.join('snap', 'snapcraft.yaml'))
        self.run_snapcraft('pull')

        with apt.Cache() as apt_cache:
            installed_version = apt_cache[self.pkg_name].candidate.version
        self.assertThat(installed_version, Equals(self.pkg_version))


class BuildPackageVersionErrorsTestCase(integration_tests.TestCase):

    def test_build_package_bad_version(self):
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'pull', 'build-package-version-bad')
        self.assertIn(
            "Version 'x.y-z' for 'hello' was not found",
            str(error.output)
        )
