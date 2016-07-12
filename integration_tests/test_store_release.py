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
import re
import subprocess
import uuid

from testtools.matchers import (
    Equals,
    FileExists,
    MatchesRegex,
)

import integration_tests

from snapcraft.tests import fixture_setup


class ReleaseTestCase(integration_tests.TestCase):

    def test_release_without_login(self):
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 
            ['release', 'test-snap', '19', 'beta'])
        self.assertIn('No valid credentials found. Have you run "snapcraft '
                      'login"?', str(error.output))

    def test_release_with_login(self):
        if not os.getenv('TEST_USER_PASSWORD', None):
            self.useFixture(fixture_setup.FakeStore())
        else:
            self.useFixture(fixture_setup.StagingStore())

        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        project_dir = 'basic'
        unique_id = uuid.uuid4().int
        new_name = 'u1test-{}'.format(unique_id)
        # The maximum size is 32 chars.
        new_version = str(unique_id)[:32]

        project_dir = self.update_name_and_version(
            project_dir, new_name, new_version)

        self.run_snapcraft('snap', project_dir)

        # Register the snap
        self.register(new_name)
        # Upload the snap
        snap_file_path = '{}_{}_{}.snap'.format(new_name, new_version, 'all')
        self.assertThat(
            os.path.join(project_dir, snap_file_path), FileExists())

        output = self.run_snapcraft(['upload', snap_file_path], project_dir)
        expected = r'.*Ready to release!.*'.format(new_name)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

        # Release it
        output = self.run_snapcraft(['release', new_name, '1', 'edge'])
        expected = r'.*The \'edge\' channel is now open.*'
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))
