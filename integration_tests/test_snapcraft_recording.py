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
from testtools.matchers import FileExists

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

    def test_prime_records_snapcraft_yaml(self):
        """Test the recorded snapcraft.yaml for a basic snap

        This snap doesn't have stage or build packages and is declared that it
        works on all architectures.
        """
        self.run_snapcraft('prime', project_dir='basic')
        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        self.assertThat(recorded_yaml_path, FileExists())

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        for key in ('build-packages', 'prime', 'stage', 'stage-packages'):
            # prime and stage come from when the yaml is loaded.
            # stage-packages comes from the annotation from state.
            source_yaml['parts']['dummy-part'].update({key: []})
        source_yaml.update(grade='stable')
        # build packages end up at the start.
        source_yaml['parts']['dummy-part'].move_to_end(
            'build-packages', last=False)

        source_yaml.update({'build-packages': []})

        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_without_arch_records_current_arch(self):
        """Test the recorded snapcraft.yaml for a basic snap

        This snap doesn't have stage or build packages and it is not declared
        that it works on all architectures, which makes it specific to the
        current architecture.
        """
        self.run_snapcraft('prime', project_dir='basic-without-arch')
        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        self.assertThat(recorded_yaml_path, FileExists())

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        for key in ('build-packages', 'prime', 'stage', 'stage-packages'):
            # prime and stage come from when the yaml is loaded.
            # stage-packages comes from the annotation from state.
            source_yaml['parts']['dummy-part'].update({key: []})
        source_yaml.update(grade='stable')
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])
        # build packages end up at the start.
        source_yaml['parts']['dummy-part'].move_to_end(
            'build-packages', last=False)

        source_yaml.update({'build-packages': []})

        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_with_global_build_packages(self):
        """Test the recorded snapcraft.yaml for a snap with build packages

        This snap declares one global build package that has undeclared
        dependencies.
        """
        self.run_snapcraft('prime', 'build-package-global')

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        part_name = 'empty-part'
        # Append the default values.
        for key in ('build-packages', 'prime', 'stage', 'stage-packages'):
            source_yaml['parts'][part_name].update({key: []})
        # build packages end up at the start.
        source_yaml['parts'][part_name].move_to_end(
            'build-packages', last=False)
        # annotate the yaml with the global build packages installed.
        source_yaml['build-packages'] = [
            '{}={}'.format(
                package,
                integration_tests.get_package_version(
                    package, self.distro_series, self.deb_arch))
            for package in ['hello', 'libc6', 'libgcc1', 'gcc-6-base']
        ]
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)


class SnapcraftRecordingPackagesTestCase(
        testscenarios.WithScenarios, SnapcraftRecordingBaseTestCase):

    scenarios = (
        ('stage-packages', {
            'packages_type': 'stage-packages',
            'expected_packages': ['gcc-6-base', 'hello']
         }),
        ('build-packages', {
            'packages_type': 'build-packages',
            'expected_packages': ['hello', 'libc6', 'libgcc1', 'gcc-6-base']
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
        for package in ['hello'] + self.expected_packages:
            self.set_package_version(
                self.packages_type,
                os.path.join('snap', 'snapcraft.yaml'), part_name, package)

        self.run_snapcraft('prime')

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)

        # Annotate the source snapcraft.yaml with the expected values.
        for key in ('prime', 'stage'):
            source_yaml['parts'][part_name].update({key: []})
        # Add the default value for the other package type.
        if 'build-packages' not in source_yaml['parts'][part_name]:
            source_yaml['parts'][part_name].update({'build-packages': []})
        if 'stage-packages' not in source_yaml['parts'][part_name]:
            source_yaml['parts'][part_name].update({'stage-packages': []})
        # build packages end up at the start.
        source_yaml['parts'][part_name].move_to_end(
            'build-packages', last=False)
        # stage packages end up at the end.
        source_yaml['parts'][part_name].move_to_end(
            'stage-packages', last=True)

        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])
        source_yaml.update({'build-packages': []})

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_without_packages_version(self):
        """Test the recorded snapcraft.yaml for a snap with packages

        This snap declares all the packages that it requires, there are
        no additional depenencies. The packages dont' specify their
        version.
        """
        self.run_snapcraft(
            'prime',
            project_dir='{}-without-dependencies'.format(self.packages_type))

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        part_name = 'part-with-{}'.format(self.packages_type)
        for key in ('prime', 'stage'):
            source_yaml['parts'][part_name].update({key: []})
        # Add the default value for the other package type.
        if 'build-packages' not in source_yaml['parts'][part_name]:
            source_yaml['parts'][part_name].update({'build-packages': []})
        if 'stage-packages' not in source_yaml['parts'][part_name]:
            source_yaml['parts'][part_name].update({'stage-packages': []})
        # build packages end up at the start.
        source_yaml['parts'][part_name].move_to_end(
            'build-packages', last=False)
        # stage packages end up at the end.
        source_yaml['parts'][part_name].move_to_end('stage-packages')
        # annotate the yaml with the package versions.
        source_yaml['parts'][part_name][self.packages_type] = [
            '{}={}'.format(
                package, integration_tests.get_package_version(
                    package, self.distro_series, self.deb_arch)) for
            package in source_yaml['parts'][part_name][self.packages_type]
        ]
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])
        source_yaml.update({'build-packages': []})

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_without_arch_records_current_arch(self):
        """Test the recorded snapcraft.yaml for a basic snap

        This snap doesn't have stage or build packages and it is not declared
        that it works on all architectures, which makes it specific to the
        current architecture.
        """
        self.run_snapcraft('prime', project_dir='basic-without-arch')
        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        self.assertThat(recorded_yaml_path, FileExists())

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        for key in ('prime', 'stage', 'stage-packages'):
            # prime and stage come from when the yaml is loaded.
            # stage-packages comes from the annotation from state.
            source_yaml['parts']['dummy-part'].update({key: []})
        source_yaml.update(grade='stable')
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_records_stage_packages_version(self):
        """Test the recorded snapcraft.yaml for a snap with stage packages

        This snap declares all the stage packages that it requires, there are
        no additional dependencies. The stage packages specify their version.
        """
        self.copy_project_to_cwd('stage-packages-without-dependencies')
        part_name = 'part-with-stage-packages'
        self.set_stage_package_version(
            os.path.join('snap', 'snapcraft.yaml'),
            part_name, package='hello')
        self.set_stage_package_version(
            os.path.join('snap', 'snapcraft.yaml'),
            part_name, package='gcc-6-base')

        self.run_snapcraft('prime')

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)

        # Safeguard assertion for the version in the source.
        self.assertThat(
            source_yaml['parts'][part_name]['stage-packages'][0],
            MatchesRegex('gcc-6-base=.+'))
        self.assertThat(
            source_yaml['parts'][part_name]['stage-packages'][1],
            MatchesRegex('hello=.+'))

        # Annotate the source snapcraft.yaml with the expected values.
        for key in ('prime', 'stage'):
            source_yaml['parts'][part_name].update({key: []})
        # stage packages end up at the end.
        source_yaml['parts'][part_name].move_to_end('stage-packages')

        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_without_stage_packages_version(self):
        """Test the recorded snapcraft.yaml for a snap with stage packages

        This snap declares all the stage packages that it requires, there are
        no additional dependencies. The stage packages don't specify their
        version.
        """
        self.run_snapcraft(
            'prime', project_dir='stage-packages-without-dependencies')

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        part_name = 'part-with-stage-packages'
        for key in ('prime', 'stage'):
            source_yaml['parts'][part_name].update({key: []})
        # stage packages end up at the end.
        source_yaml['parts'][part_name].move_to_end('stage-packages')
        # annotate the yaml with the staged versions.
        source_yaml['parts'][part_name]['stage-packages'] = [
            '{}={}'.format(
                package, integration_tests.get_package_version(
                    package, self.distro_series, self.deb_arch)) for
            package in source_yaml['parts'][part_name]['stage-packages']
        ]
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])
        source_yaml.update({'build-packages': []})

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_with_stage_package_missing_dependecy(self):
        """Test the recorded snapcraft.yaml for a snap with stage packages

        This snap declares one stage packages that has one undeclared
        dependency.
        """
        self.copy_project_to_cwd('stage-package-missing-dependency')
        part_name = 'part-with-stage-package'
        self.set_stage_package_version(
            os.path.join('snap', 'snapcraft.yaml'), part_name, package='hello')
        self.run_snapcraft('prime')

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        part_name = 'part-with-stage-package'
        # Append the default values.
        for key in ('prime', 'stage'):
            source_yaml['parts'][part_name].update({key: []})
        # stage packages end up at the end.
        source_yaml['parts'][part_name].move_to_end('stage-packages')
        # annotate the yaml with the installed version.
        source_yaml['parts'][part_name]['stage-packages'].insert(
            0, 'gcc-6-base={}'.format(integration_tests.get_package_version(
                'gcc-6-base', self.distro_series, self.deb_arch)))
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_with_packages_missing_dependency(self):
        """Test the recorded snapcraft.yaml for a snap with packages

        This snap declares one package that has undeclared dependencies.
        """
        self.copy_project_to_cwd('{}-missing-dependency'.format(
            self.packages_type))
        part_name = 'part-with-{}'.format(self.packages_type)
        self.set_package_version(
            self.packages_type,
            os.path.join('snap', 'snapcraft.yaml'), part_name, package='hello')
        self.run_snapcraft('prime')

        # Annotate the source snapcraft.yaml with the expected values.
        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        # Append the default values.
        for key in ('prime', 'stage'):
            source_yaml['parts'][part_name].update({key: []})
        # Add the default value for the other package type.
        if 'build-packages' not in source_yaml['parts'][part_name]:
            source_yaml['parts'][part_name].update({'build-packages': []})
        if 'stage-packages' not in source_yaml['parts'][part_name]:
            source_yaml['parts'][part_name].update({'stage-packages': []})
        # build packages end up at the start.
        source_yaml['parts'][part_name].move_to_end(
            'build-packages', last=False)
        # stage packages end up at the end.
        source_yaml['parts'][part_name].move_to_end('stage-packages')
        # annotate the yaml with the installed version.
        source_yaml['parts'][part_name][self.packages_type] = [
            '{}={}'.format(
                package,
                integration_tests.get_package_version(
                    package, self.distro_series, self.deb_arch))
            for package in self.expected_packages
        ]
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)
