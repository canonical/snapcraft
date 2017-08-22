# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017 Canonical Ltd
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
import yaml

import integration_tests


class CmakePluginTestCase(integration_tests.TestCase):

    def test_stage_cmake_plugin(self):
        self.run_snapcraft('stage', 'cmake-hello')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'cmake-hello'))
        self.assertEqual("It's a CMake world\n", binary_output)

    def test_build_cmake_in_builddir(self):
        self.copy_project_to_cwd('cmake-hello')
        with open('snapcraft.yaml') as snapcraft_yaml_file:
            snapcraft_yaml = yaml.load(snapcraft_yaml_file)
        snapcraft_yaml['parts']['cmake-project']['prepare'] = (
            'sed -i "s/CMake world/CMake build dir world/" test.c')
        with open('snapcraft.yaml', 'w') as snapcraft_yaml_file:
            yaml.dump(snapcraft_yaml, snapcraft_yaml_file)

        self.run_snapcraft('build')
        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(
                'parts', 'cmake-project', self.build_dir, 'cmake_build',
                'cmake-hello'))
        self.assertEqual("It's a CMake build dir world\n", binary_output)
