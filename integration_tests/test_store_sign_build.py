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
import fixtures
import shutil
import integration_tests

from testtools.matchers import FileExists

class SignBuildTestCase(integration_tests.StoreTestCase):

    def setUp(self):
        super().setUp()
        keys_dir = os.path.join(os.path.dirname(__file__), 'keys')
        temp_keys_dir = os.path.join(self.path, '.snap', 'gnupg')
        shutil.copytree(keys_dir, temp_keys_dir)
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAP_GNUPG_HOME', temp_keys_dir))
        self.snap_path = 'basic_0.1_all.snap'

    def test_unsuccessful_sign_build(self):
        project_dir = 'basic'
        self.addCleanup(self.logout)
        self.login()

        self.run_snapcraft('snap', project_dir)
        os.chdir(project_dir)
        self.assertThat(self.snap_path, FileExists())
        message = 'Failed to sign build assertion basic_0.1_all.snap-build.'
        output = self.sign_build(self.snap_path)
        self.assertEqual(output, message)

    def test_successful_sign_build_local(self):
        project_dir = 'basic'
        self.addCleanup(self.logout)
        self.login()

        self.run_snapcraft('snap', project_dir)
        os.chdir(project_dir)
        self.assertThat(self.snap_path, FileExists())
        message = 'Build assertion basic_0.1_all.snap-build saved to disk.'
        output = self.sign_build(self.snap_path)
        self.assertEqual(output, message)

    def test_successful_sign_build_push(self):
        project_dir = 'basic'
        self.addCleanup(self.logout)
        self.login()

        self.run_snapcraft('snap', project_dir)
        os.chdir(project_dir)
        self.assertThat(self.snap_path, FileExists())
        self.assertEqual(0, self.register_key('default'))
        message = 'Build assertion basic_0.1_all.snap-build pushed.'
        output = self.sign_build(self.snap_path)
        self.assertEqual(output, message)
