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

import integration_tests


class GoPluginTestCase(integration_tests.TestCase):

    def test_stage_go_plugin(self):
        project_dir = 'simple-go'
        self.run_snapcraft('stage', project_dir)

        binary_output = subprocess.check_output(
            os.path.join('stage', 'bin', project_dir),
            universal_newlines=True, cwd=project_dir)
        self.assertEqual('Hello snapcrafter\n', binary_output)

    def test_building_multiple_main_packages(self):
        project_dir = 'multiple-main-go-packages'
        self.run_snapcraft('stage', project_dir)
