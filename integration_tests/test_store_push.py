# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
    FileExists,
    MatchesRegex,
)

import integration_tests


class PushTestCase(integration_tests.StoreTestCase):

    def test_push_without_login(self):
        self.run_snapcraft('snap', 'basic')
        snap_file_path = 'basic_0.1_{}.snap'.format('all')
        self.assertThat(snap_file_path, FileExists())

        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['push', snap_file_path])
        self.assertIn('No valid credentials found. Have you run "snapcraft '
                      'login"?', str(error.output))

    def test_push_with_login(self):
        # Make a snap
        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        unique_id = uuid.uuid4().int
        new_name = 'u1test-{}'.format(unique_id)
        # The maximum size is 32 chars.
        new_version = str(unique_id)[:32]

        self.copy_project_to_cwd('basic')
        self.update_name_and_version(new_name, new_version)

        self.run_snapcraft('snap')

        # Register the snap
        self.register(new_name)
        # Upload the snap
        snap_file_path = '{}_{}_{}.snap'.format(new_name, new_version, 'all')
        self.assertThat(
            os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(['push', snap_file_path])
        expected = r'.*Ready to release!.*'.format(new_name)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

    def test_push_and_release(self):
        # Make a snap
        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        unique_id = uuid.uuid4().int
        new_name = 'u1test-{}'.format(unique_id)
        # The maximum size is 32 chars.
        new_version = str(unique_id)[:32]

        self.copy_project_to_cwd('basic')
        self.update_name_and_version(new_name, new_version)

        self.run_snapcraft('snap')

        # Register the snap
        self.register(new_name)
        # Upload the snap
        snap_file_path = '{}_{}_{}.snap'.format(new_name, new_version, 'all')
        self.assertThat(
            os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(
            ['push', snap_file_path, '--release', 'edge'])
        expected = r'.*Ready to release!.*'.format(new_name)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))
        expected = r'.*The \'edge\' channel is now open.*'
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

    def test_push_with_deprecated_upload(self):
        # Make a snap
        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        unique_id = uuid.uuid4().int
        new_name = 'u1test-{}'.format(unique_id)
        # The maximum size is 32 chars.
        new_version = str(unique_id)[:32]

        self.copy_project_to_cwd('basic')
        self.update_name_and_version(new_name, new_version)

        self.run_snapcraft('snap')

        # Register the snap
        self.register(new_name)
        # Upload the snap
        snap_file_path = '{}_{}_{}.snap'.format(new_name, new_version, 'all')
        self.assertThat(
            os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(['upload', snap_file_path])
        expected = r'.*Ready to release!.*'.format(new_name)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))
