# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

from testtools.matchers import FileExists

import integration_tests


class PullPropertiesTestCase(integration_tests.TestCase):

    def test_pull(self):
        self.assert_expected_pull_state('local-plugin-pull-properties')

    def test_pull_legacy_pull_properties(self):
        self.assert_expected_pull_state('local-plugin-legacy-pull-properties')

    def assert_expected_pull_state(self, project_dir):
        self.run_snapcraft('pull', project_dir)

        state_file = os.path.join(
            project_dir, 'parts', 'x-local-plugin', 'state', 'pull')
        self.assertThat(state_file, FileExists())
        with open(state_file) as f:
            state = yaml.load(f)

        # Verify that the correct schema dependencies made it into the state.
        self.assertTrue('foo' in state.schema_properties)
        self.assertTrue('stage-packages' in state.schema_properties)

        # Verify that the contents of the dependencies made it in as well.
        self.assertTrue('foo' in state.properties)
        self.assertTrue('stage-packages' in state.properties)
        self.assertEqual('bar', state.properties['foo'])
        self.assertEqual(['curl'], state.properties['stage-packages'])
