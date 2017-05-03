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
from testtools.matchers import FileExists

import integration_tests


class SnapcraftRecordingTestCase(integration_tests.TestCase):

    def test_prime_records_snapcraft_yaml(self):
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAPCRAFT_BUILD_INFO', '1'))

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
