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

import subprocess

import fixtures
from testtools.matchers import FileExists

from snapcraft.tests import integration


class ContainerBuildsTestCase(integration.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable('SNAPCRAFT_CONTAINER_BUILDS', '1'))

    def run_snapcraft(self, project_dir):
        if project_dir:
            self.copy_project_to_cwd(project_dir)

        command = [self.snapcraft_command, '--debug']
        popen = subprocess.Popen(
            command, stdout=subprocess.PIPE, universal_newlines=True)
        for line in iter(popen.stdout.readline, ''):
            print(line)
        popen.stdout.close()
        return_code = popen.wait()
        if return_code:
            raise subprocess.CalledProcessError(return_code, command)

    def test_container_build(self):
        self.run_snapcraft('basic')

        snap_file_path = 'basic_0.1_all.snap'
        self.assertThat(snap_file_path, FileExists())
