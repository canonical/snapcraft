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
import yaml

import fixtures
import testscenarios

import snapcraft
import integration_tests


class SnapcraftRecordingBaseTestCase(integration_tests.TestCase):
    """Test that the prime step records an annotated snapcraft.yaml

    The annotated file will be in prime/snap/snapcraft.yaml.

    """

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))


class SnapcraftRecordingTestCase(SnapcraftRecordingBaseTestCase):

    def test_prime_with_architectures(self):
        """Test the recorded snapcraft.yaml for a basic snap

        This snap doesn't have stage or build packages and is declared that it
        works on all architectures.
        """
        self.run_snapcraft('prime', project_dir='basic')

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml['architectures'], ['all'])

    def test_prime_without_architectures_records_current_arch(self):
        """Test the recorded snapcraft.yaml for a basic snap

        This snap doesn't have stage or build packages and it is not declared
        that it works on all architectures, which makes it specific to the
        current architecture.
        """
        self.run_snapcraft('prime', project_dir='basic-without-arch')

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(
            recorded_yaml['architectures'],
            [snapcraft.ProjectOptions().deb_arch])

    def test_prime_with_stage_package_missing_dependecy(self):
        """Test the recorded snapcraft.yaml for a snap with stage packages

        This snap declares one stage package that has one undeclared
        dependency.
        """
        self.copy_project_to_cwd('stage-package-missing-dependency')
        part_name = 'part-with-stage-package'
        self.set_stage_package_version(
            os.path.join('snap', 'snapcraft.yaml'), part_name, package='hello')
        self.run_snapcraft('prime')

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        part_name = 'part-with-stage-package'
        expected_stage_packages = (
            source_yaml['parts'][part_name]['stage-packages'])
        # Add the dependency recorded by snapcraft.
        expected_stage_packages.insert(
            0, 'gcc-6-base={}'.format(integration_tests.get_package_version(
                'gcc-6-base', self.distro_series, self.deb_arch)))

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(
            recorded_yaml['parts'][part_name]['stage-packages'],
            expected_stage_packages)


class SnapcraftRecordingPackagesTestCase(
        testscenarios.WithScenarios, SnapcraftRecordingBaseTestCase):

    scenarios = (
        ('stage-packages', {
            'packages_type': 'stage-packages'
         }),
        ('build-packages', {
            'packages_type': 'build-packages'
         })
    )

    def test_prime_records_packages_version(self):
        """Test the recorded snapcraft.yaml for a snap with packages

        This snap declares all the packages that it requires, there are
        no additional dependencies. The packages specify their version.
        """
        self.copy_project_to_cwd(
            '{}-without-dependencies'.format(self.packages_type))
        part_name = 'part-with-{}'.format(self.packages_type)
        self.set_package_version(
            self.packages_type,
            os.path.join('snap', 'snapcraft.yaml'),
            part_name, package='hello')
        self.set_package_version(
            self.packages_type,
            os.path.join('snap', 'snapcraft.yaml'),
            part_name, package='gcc-6-base')

        self.run_snapcraft('prime')

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(
            recorded_yaml['parts'][part_name][self.packages_type],
            source_yaml['parts'][part_name][self.packages_type])

    def test_prime_without_packages_version(self):
        """Test the recorded snapcraft.yaml for a snap with packages

        This snap declares all the packages that it requires, there are
        no additional dependencies. The packages don't specify their
        version.
        """
        self.run_snapcraft(
            'prime',
            project_dir='{}-without-dependencies'.format(self.packages_type))

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        part_name = 'part-with-{}'.format(self.packages_type)
        # Add the versions recorded by snapcraft.
        expected_stage_packages = [
            '{}={}'.format(
                package, integration_tests.get_package_version(
                    package, self.distro_series, self.deb_arch)) for
            package in source_yaml['parts'][part_name][self.packages_type]
        ]

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(
            recorded_yaml['parts'][part_name][self.packages_type],
            expected_stage_packages)
