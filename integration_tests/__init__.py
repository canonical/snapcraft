# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
import subprocess

import shutil
import testtools

from snapcraft.tests import fixture_setup


class TestCase(testtools.TestCase):

    def setUp(self):
        super().setUp()
        snapcraft_bin = os.getenv('SNAPCRAFT', 'snapcraft')
        self.snapcraft_command = os.path.join(
            os.getcwd(), 'bin', snapcraft_bin)
        self.snaps_dir = os.path.join(os.path.dirname(__file__), 'snaps')

        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

    def run_snapcraft(self, command, project_dir=None):
        if isinstance(command, str):
            command = [command]
        if project_dir:
            if not os.path.exists(project_dir):
                cwd = self.copy_project_to_tmp(project_dir)
            else:
                cwd = os.path.join(self.path, project_dir)
        else:
            cwd = None
        return subprocess.check_output(
            [self.snapcraft_command] + command, cwd=cwd,
            stderr=subprocess.STDOUT, universal_newlines=True)

    def copy_project_to_tmp(self, project_dir):
        tmp_project_dir = os.path.join(self.path, project_dir)
        shutil.copytree(
            os.path.join(self.snaps_dir, project_dir), tmp_project_dir)
        return tmp_project_dir
