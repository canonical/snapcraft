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

import fileinput
import os
import subprocess
import uuid

from testtools.matchers import (
    FileExists,
)

import integration_tests
import snapcraft
from snapcraft.tests import fixture_setup


class UploadTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    def _update_version(self, project_dir, version=None):
        # Change to a random version.
        # The maximum size is 32 chars.
        if version is None:
            version = str(uuid.uuid4().int)[:32]
        updated_project_dir = self.copy_project_to_tmp(project_dir)
        yaml_file = os.path.join(project_dir, 'snapcraft.yaml')
        for line in fileinput.input(yaml_file, inplace=True):
            if 'version: ' in line:
                print('version: ' + version)
            else:
                print(line)
        return updated_project_dir

    def test_upload_without_snap(self):
        raised = self.assertRaises(subprocess.CalledProcessError,
                                   self.run_snapcraft, ['upload'])
        self.assertTrue(raised.output.startswith('Usage:\n'))

    def test_upload_without_login(self):
        project_dir = 'assemble'
        self.run_snapcraft('snap', project_dir)
        snap_file_path = 'assemble_1.0_{}.snap'.format(self.deb_arch)
        os.chdir(project_dir)
        self.assertThat(snap_file_path, FileExists())

        output = self.run_snapcraft(['upload', snap_file_path])

        self.assertIn('No valid credentials found. Have you run "snapcraft '
                      'login"?', output)

    def test_upload_with_login(self):
        self.useFixture(fixture_setup.StagingStore())

        # Make a snap
        project_dir = 'basic'
        self.addCleanup(self.logout)
        self.login()

        # Change to a random version.
        # The maximum size is 32 chars.
        new_version = str(uuid.uuid4().int)[:32]
        project_dir = self._update_version(project_dir, new_version)

        self.run_snapcraft('snap', project_dir)

        # Upload the snap
        snap_file_path = 'basic_{}_{}.snap'.format(new_version, self.deb_arch)
        self.assertThat(
            os.path.join(project_dir, snap_file_path), FileExists())

        output = self.run_snapcraft(['upload', snap_file_path], project_dir)
        self.assertIn(
            'Application uploaded successfully (as revision ', output)
        self.assertIn('Please check out the application at: ', output)
