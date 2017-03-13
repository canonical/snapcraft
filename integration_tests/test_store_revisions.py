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

from testtools.matchers import Contains, FileExists, MatchesRegex

import integration_tests


class RevisionsTestCase(integration_tests.StoreTestCase):

    def test_revisions_without_login(self):
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['revisions', 'test-snap'])
        self.assertIn('No valid credentials found. Have you run "snapcraft '
                      'login"?', str(error.output))

    def test_revisions_with_login_wrong_snap(self):
        self.addCleanup(self.logout)
        self.login()

        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['revisions', 'mysnap'])
        self.assertIn(
            "Snap 'mysnap' was not found in '16' series.", str(error.output))

    def test_revisions_with_login_bad_snap_with_series(self):
        self.addCleanup(self.logout)
        self.login()

        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['revisions', 'mysnap', '--series=16'])
        self.assertIn(
            "Snap 'mysnap' was not found in '16' series.", str(error.output))

    def test_revisions_with_login_bad_snap_with_arch(self):
        self.addCleanup(self.logout)
        self.login()

        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['revisions', 'mysnap', '--arch=i386'])
        self.assertIn(
            "Snap 'mysnap' for 'i386' was not found in '16' series.",
            str(error.output))

    @unittest.skipUnless(
        os.getenv('TEST_STORE', 'fake') == 'fake', 'Skip fake store.')
    def test_revisions_fake_store(self):
        self.addCleanup(self.logout)
        self.login()

        output = self.run_snapcraft(['revisions', 'basic'])
        expected = '\n'.join((
            'Rev.    Uploaded              Arch    Version    Channels',
            '2       2016-09-27T19:23:40Z  i386    2.0.1      -',
            '1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge'
        ))
        self.assertThat(output, Contains(expected))

    @unittest.skipUnless(
        os.getenv('TEST_STORE', 'fake') == 'fake', 'Skip fake store.')
    def test_list_revisions_fake_store(self):
        self.addCleanup(self.logout)
        self.login()

        output = self.run_snapcraft(['list-revisions', 'basic'])
        expected = '\n'.join((
            'Rev.    Uploaded              Arch    Version    Channels',
            '2       2016-09-27T19:23:40Z  i386    2.0.1      -',
            '1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge'
        ))
        self.assertThat(output, Contains(expected))

    @unittest.skipUnless(
        os.getenv('TEST_STORE', 'fake') == 'fake', 'Skip fake store.')
    def test_history_fake_store(self):
        self.addCleanup(self.logout)
        self.login()

        output = self.run_snapcraft(['history', 'basic'])
        expected = '\n'.join((
            'Rev.    Uploaded              Arch    Version    Channels',
            '2       2016-09-27T19:23:40Z  i386    2.0.1      -',
            '1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge'
        ))
        self.assertThat(output, Contains(expected))

        self.assertThat(output, Contains(
            "DEPRECATED: The 'history' command has "
            "been replaced by 'list-revisions'."))

    @unittest.skipUnless(
        os.getenv('TEST_STORE', 'fake') == 'staging', 'Skip staging store.')
    def test_revisions_staging_store(self):
        self.addCleanup(self.logout)
        self.login()

        # Build a random snap, register, push and release it.
        self.copy_project_to_cwd('basic')
        unique_id = uuid.uuid4().int
        name = 'u1test-{}'.format(unique_id)
        version = '1.0'
        self.update_name_and_version(name, version)
        self.run_snapcraft('snap')
        snap_path = '{}_{}_{}.snap'.format(name, version, 'all')
        self.assertThat(snap_path, FileExists())
        self.register(name)
        self.assertEqual(0, self.push(snap_path, release='candidate,beta'))

        output = self.run_snapcraft(['revisions', name])

        datetime_re = '\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z'
        expected = '\n'.join((
            '.*',
            'Rev.    Uploaded              Arch       Version    Channels',
            '1       {datetime_re}  Arch: All  1          candidate\*, beta\*'
            '.*',
        )).format(datetime_re=datetime_re)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))
