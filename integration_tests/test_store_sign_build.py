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
import fixtures
import shutil
import uuid

from testtools.matchers import (
    Contains,
    FileContains,
    FileExists,
    Not,
)

import integration_tests


class SignBuildTestCase(integration_tests.StoreTestCase):

    def setUp(self):
        super().setUp()
        keys_dir = os.path.join(os.path.dirname(__file__), 'keys')
        temp_keys_dir = os.path.join(self.path, '.snap', 'gnupg')
        shutil.copytree(keys_dir, temp_keys_dir)
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAP_GNUPG_HOME', temp_keys_dir))
        self.project = 'basic'
        self.snap_path = '{}_0.1_all.snap'.format(self.project)
        self.snap_build_path = '{}-build'.format(self.snap_path)

    def test_unsuccessful_sign_build_no_login(self):
        self.run_snapcraft('snap', self.project)
        self.assertThat(self.snap_path, FileExists())

        status = self.sign_build(
            self.snap_path, local=True, expect_success=False)
        self.assertEqual(1, status)
        self.assertThat(self.snap_build_path, Not(FileExists()))

    def test_successful_sign_build_local(self):
        self.addCleanup(self.logout)
        self.login()

        # Build a random snap and register it.
        unique_id = uuid.uuid4().int
        name = 'u1test-{}'.format(unique_id)
        version = str(unique_id)[:32]
        self.copy_project_to_cwd(self.project)
        self.update_name_and_version(name, version)
        self.run_snapcraft('snap')
        snap_path = '{}_{}_{}.snap'.format(name, version, 'all')
        self.assertThat(snap_path, FileExists())
        self.register(name)

        status = self.sign_build(snap_path, local=True)
        self.assertEqual(0, status)

        snap_build_path = '{}-build'.format(snap_path)
        self.assertThat(snap_build_path, FileExists())
        self.assertThat(snap_build_path,
                        FileContains(matcher=Contains('type: snap-build')))

    def test_unsuccessful_sign_build_push_no_login(self):
        self.run_snapcraft('snap', self.project)
        self.assertThat(self.snap_path, FileExists())

        status = self.sign_build(self.snap_path, expect_success=False)
        self.assertEqual(1, status)
        self.assertThat(self.snap_build_path, Not(FileExists()))

    def test_successful_sign_build_push(self):
        if os.getenv('TEST_STORE', 'fake') != 'fake':
            # https://bugs.launchpad.net/bugs/1621441
            self.skipTest(
                'Cannot push signed assertion against staging/production '
                'until we have a way to delete them again.')

        self.run_snapcraft('snap', self.project)
        self.assertThat(self.snap_path, FileExists())

        self.assertEqual(0, self.register_key('default'))
        self.addCleanup(self.logout)
        self.login()

        status = self.sign_build(self.snap_path)
        self.assertEqual(0, status)
        self.assertThat(self.snap_build_path, FileExists())
        self.assertThat(self.snap_build_path,
                        FileContains(matcher=Contains('type: snap-build')))
