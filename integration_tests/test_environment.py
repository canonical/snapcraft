# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017 Canonical Ltd
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

from testtools.matchers import FileContains

import integration_tests


class EnvironmentTestCase(integration_tests.TestCase):

    def test_environment(self):
        self.run_snapcraft('stage', 'snapcraft-environment')

        stage_dir = os.path.join(self.path, 'stage')
        part_install_dir = os.path.join(self.path,
                                        'parts', 'env', 'install')

        test_name = os.path.join(stage_dir, 'test_name')
        self.assertThat(test_name, FileContains('test-environment'))

        test_version = os.path.join(stage_dir, 'test_version')
        self.assertThat(test_version, FileContains('0.1'))

        test_stage = os.path.join(stage_dir, 'test_stage')
        self.assertThat(test_stage, FileContains(stage_dir))

        test_part_install = os.path.join(stage_dir, 'test_part_install')
        self.assertThat(test_part_install, FileContains(part_install_dir))

    def test_project_environment_within_snapcraft(self):
        """Replace the SNAPCRAFT_PROJECT_.* occurrences in snapcraft.yaml

        If the correct environment SNAPCRAFT_PROJECT values aren't replaced
        this test will fail its pull step."""
        self.run_snapcraft('pull', 'snapcraft-key-values')

    def test_snapcraft_stage_env_replacement(self):
        self.run_snapcraft('stage', 'stage_env')

    def test_stage_cmake_plugin_with_replace(self):
        """Replace SNAPCRAFT_PART_INSTALL in the part's attributes"""
        self.run_snapcraft('stage', 'simple-cmake-replace')

        binary_output = subprocess.check_output([
            os.path.join('stage', 'bin', 'simple-cmake-replace')])
        path = os.path.join(self.path, 'parts', 'cmake-project', 'install')
        self.assertEqual(
            "When I was built I was installed to {}\n".format(path),
            binary_output.decode('utf-8'))
