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
from snapcraft.tests import fixture_setup


def _get_deb_arch():
    return _run_dpkg_architecture('-qDEB_BUILD_ARCH')


def _run_dpkg_architecture(arg):
    return subprocess.check_output(
        ['dpkg-architecture', arg], universal_newlines=True).strip()


class UploadTestCase(integration_tests.TestCase):

    def test_upload_without_login(self):
        project_dir = 'assemble'
        output = self.run_snapcraft('upload', project_dir)
        os.chdir(project_dir)

        snap_file_path = 'assemble_1.0_{}.snap'.format(_get_deb_arch())
        self.assertThat(snap_file_path, FileExists())

        self.assertIn('Snap assemble_1.0_amd64.snap not found. Running snap '
                      'step to create it.', output)
        self.assertIn('Upload failed', output)
        self.assertIn('No valid credentials found. Have you run "snapcraft '
                      'login"?', output)

    def test_upload_with_login(self):
        self.useFixture(fixture_setup.StagingStore())

        project_dir = 'basic'
        self.addCleanup(self.logout)
        self.login()

        project_dir = self.copy_project_to_tmp(project_dir)
        yaml_file = os.path.join(project_dir, 'snapcraft.yaml')
        for line in fileinput.input(yaml_file, inplace=True):
            if 'version: ' in line:
                # Change to a random version.
                # The maximum size is 32 chars.
                print('version: ' + str(uuid.uuid4().int)[:32])
            else:
                print(line)

        output = self.run_snapcraft('upload', project_dir)
        self.assertIn(
            'Application uploaded successfully (as revision ', output)
        self.assertIn('Please check out the application at: ', output)
