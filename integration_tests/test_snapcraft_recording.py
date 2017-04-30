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
from testtools.matchers import (
    FileExists,
    MatchesRegex
)

import snapcraft
import integration_tests


class SnapcraftRecordingTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))

    def test_prime_records_snapcraft_yaml(self):
        self.run_snapcraft('prime', project_dir='basic')
        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        self.assertThat(recorded_yaml_path, FileExists())

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        # Append the default values.
        for key in ('prime', 'stage'):
            source_yaml['parts']['dummy-part'].update({key: []})
        source_yaml.update(grade='stable')

        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_without_arch_records_current(self):
        self.run_snapcraft('prime', project_dir='basic-without-arch')
        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        self.assertThat(recorded_yaml_path, FileExists())

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        # Append the default values.
        for key in ('prime', 'stage'):
            source_yaml['parts']['dummy-part'].update({key: []})
        source_yaml.update(grade='stable')
        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_records_stage_package(self):
        self.run_snapcraft('prime', project_dir='stage-package')
        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        self.assertThat(recorded_yaml_path, FileExists())

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)
        # Append the default values.
        for key in ('prime', 'stage'):
            source_yaml['parts']['hello'].update({key: []})
        # stage packages end up at the end.
        source_yaml['parts']['hello'].move_to_end('stage-packages')

        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)

    def test_prime_records_stage_package_with_version(self):
        self.copy_project_to_cwd('stage-package')
        self.set_stage_package_version(
            os.path.join('snap', 'snapcraft.yaml'),
            part='hello', package='hello')

        self.run_snapcraft('prime')
        recorded_yaml_path = os.path.join(
            self.prime_dir, 'snap', 'snapcraft.yaml')
        self.assertThat(recorded_yaml_path, FileExists())

        with open(os.path.join('snap', 'snapcraft.yaml')) as source_yaml_file:
            source_yaml = yaml.load(source_yaml_file)

        # Safeguard assertion for the version in the source.
        self.assertThat(
            source_yaml['parts']['hello']['stage-packages'][0],
            MatchesRegex('hello=.+'))

        # Append the default values.
        for key in ('prime', 'stage'):
            source_yaml['parts']['hello'].update({key: []})
        # stage packages end up at the end.
        source_yaml['parts']['hello'].move_to_end('stage-packages')

        source_yaml.update(
            architectures=[snapcraft.ProjectOptions().deb_arch])

        with open(recorded_yaml_path) as recorded_yaml_file:
            recorded_yaml = yaml.load(recorded_yaml_file)

        self.assertEqual(recorded_yaml, source_yaml)
