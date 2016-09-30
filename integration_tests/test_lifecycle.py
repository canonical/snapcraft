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

import integration_tests


class LifecycleTestCase(integration_tests.TestCase):

    def test_stage_cmake_plugin_with_replace(self):
        project_dir = 'simple-cmake-replace'
        self.run_snapcraft('stage', project_dir)

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(
                'stage', os.path.relpath(os.getcwd(), '/'),  project_dir,
                'parts', 'cmake-project', 'install', 'bin',
                'simple-cmake-replace'),
            cwd=project_dir)
        self.assertEqual("It's a CMake world\n", binary_output)
