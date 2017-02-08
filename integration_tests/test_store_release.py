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
import unittest
import uuid

from testtools.matchers import (
    FileExists,
    MatchesRegex,
)

import integration_tests


def _can_test_track():
    test_tracks = os.getenv('TEST_SNAP_WITH_TRACKS',
                            'test-snapcraft-tracks') == 'test-snapcraft-tracks'
    integration_user = (
        os.getenv('TEST_USER_EMAIL') == 'u1test+snapcraft@canonical.com'
    )

    return test_tracks and integration_user


class ReleaseTestCase(integration_tests.StoreTestCase):

    def test_release_without_login(self):
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft,
            ['release', 'test-snap', '19', 'beta'])
        self.assertIn('No valid credentials found. Have you run "snapcraft '
                      'login"?', str(error.output))

    def test_release_with_login(self):
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

        # Release it
        output = self.run_snapcraft(['release', new_name, '1', 'edge'])
        expected = r'.*The \'edge\' channel is now open.*'
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

    @unittest.skipUnless(_can_test_track(), 'Skip CI only test')
    def test_release_with_login_multiarch(self):
        self.addCleanup(self.logout)
        self.login()

        # Change to a pre-set name and version.
        unique_id = uuid.uuid4().int
        new_name = os.getenv('TEST_SNAP_WITH_TRACKS', 'test-snapcraft-tracks')
        # The maximum size is 32 chars.
        new_version = str(unique_id)[:32]

        self.copy_project_to_cwd('multiarch')
        self.update_name_and_version(new_name, new_version)

        self.run_snapcraft('snap')

        # The snap name is pre-registered

        # Upload the snap
        snap_file_path = '{}_{}_{}.snap'.format(new_name, new_version, 'multi')
        self.assertThat(
            os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(['upload', snap_file_path])
        expected = r'.*Ready to release!.*'.format(new_name)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

        # Release it
        output = self.run_snapcraft(['release', new_name, '1', '0.1/edge'])

        expected = r'.*all     0.1      16        stable     -          -.*'
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

    @unittest.skipUnless(_can_test_track(), 'Skip CI only test')
    def test_release_with_login_arm(self):
        self.addCleanup(self.logout)
        self.login()

        # Change to a pre-set name and version.
        unique_id = uuid.uuid4().int
        new_name = os.getenv('TEST_SNAP_WITH_TRACKS', 'test-snapcraft-tracks')
        # The maximum size is 32 chars.
        new_version = str(unique_id)[:32]

        self.copy_project_to_cwd('arm')
        self.update_name_arch_and_version(new_name, 'armhf', new_version)

        self.run_snapcraft('snap')

        # The snap name is pre-registered

        # Upload the snap
        snap_file_path = '{}_{}_{}.snap'.format(new_name, new_version, 'armhf')
        self.assertThat(
            os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(['upload', snap_file_path])
        expected = r'.*Ready to release!.*'.format(new_name)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

        output = self.run_snapcraft(['release', new_name, '1', '0.1/edge'])
        expected = r'.*all     0.1      16        stable     -          -.*'
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

        self.run_snapcraft('clean')
        self.update_name_arch_and_version(new_name, 'amd64', new_version)
        self.run_snapcraft('snap')

        # Release it
        output = self.run_snapcraft(['release', new_name, '2', '0.1/edge'])
        expected = r'.*armhf   0.1      16        stable     '
        r'-                                 -.*'
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))
