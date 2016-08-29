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

from testtools.matchers import Contains

import integration_tests


class AfterTestCase(integration_tests.TestCase):

    def test_stage_dependencies(self):
        project_dir = 'dependencies'
        self.run_snapcraft('stage', project_dir)

        self.assertTrue(
            os.access(
                os.path.join(project_dir, 'stage', 'bin', 'p3'),
                os.X_OK))

    def test_build_with_circular_dependencies(self):
        project_dir = self.copy_project_to_tmp('dependencies')
        os.chdir(project_dir)

        snapcraft_yaml_path = os.path.join(project_dir, 'snapcraft.yaml')
        with open(snapcraft_yaml_path, 'r') as snapcraft_yaml:
            snapcraft_yaml_contents = snapcraft_yaml.read()
        with open(snapcraft_yaml_path, 'w') as snapcraft_yaml:
            snapcraft_yaml.write(
                snapcraft_yaml_contents.replace(
                    'p1:',
                    'p1:\n'
                    '    after: [p3]'))

        # We update here to get a clean log/stdout later
        self.run_snapcraft('update', project_dir)

        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'build', project_dir)

        self.assertEqual(1, exception.returncode)
        expected = (
            'Issue detected while analyzing snapcraft.yaml: '
            'circular dependency chain found in parts definition\n')
        self.assertThat(exception.output, Contains(expected))

    def test_build_with_missing_dependencies(self):
        project_dir = self.copy_project_to_tmp('dependencies')
        os.chdir(project_dir)

        snapcraft_yaml_path = os.path.join(project_dir, 'snapcraft.yaml')
        with open(snapcraft_yaml_path, 'r') as snapcraft_yaml:
            snapcraft_yaml_contents = snapcraft_yaml.read()
        wrong_contents = snapcraft_yaml_contents.replace(
            '    after: [p1]\n',
            '')
        wrong_contents = wrong_contents.replace(
            '    after: [p2]\n',
            '')
        with open(snapcraft_yaml_path, 'w') as snapcraft_yaml:
            snapcraft_yaml.write(wrong_contents)

        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'build')

        self.assertEqual(1, exception.returncode)

    def test_pull_with_tree_of_dependencies(self):
        project_dir = os.path.join('simple-circle', 'tree')
        self.run_snapcraft('pull', project_dir)

    def test_pull_with_circular_dependencies(self):
        project_dir = os.path.join('simple-circle', 'circle')
        self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'pull', project_dir)
