# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

from testtools.matchers import FileExists

import integration_tests


class CleanbuildTestCase(integration_tests.TestCase):

    def run_snapcraft_cleanbuild(self, project_dir):
        if project_dir:
            self.copy_project_to_cwd(project_dir)

        command = [sudo, self.snapcraft_command, '-d', 'cleanbuild']
        popen = subprocess.Popen(command, stdout=subprocess.PIPE, universal_newlines=True)
        for stdout_line in iter(popen.stdout.readline, ''):
            yield stdout_line
        popen.stdout.close()
        return_code = popen.wait()
        if return_code:
            raise subprocess.CalledProcessError(return_code, command)

    def test_cleanbuild(self):
        for line in self.run_snapcraft_cleanbuild('assemble'):
            print(line)

        snap_source_path = 'assemble_1.0_source.tar.bz2'
        self.assertThat(snap_source_path, FileExists())

        snap_file_path = 'assemble_1.0_{}.snap'.format(self.deb_arch)
        self.assertThat(snap_file_path, FileExists())
