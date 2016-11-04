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
from glob import glob

from testtools.matchers import (
    DirExists,
    FileExists
)

import integration_tests


class PythonPluginTestCase(integration_tests.TestCase):

    def test_pull_with_pip_requirements_file(self):
        project_dir = 'pip-requirements-file'
        self.run_snapcraft('build', project_dir)
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python2', 'install', 'lib',
                'python2*', 'site-packages', 'argparse.py'))[0],
            FileExists())
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'lib',
                'python3*', 'site-packages', 'argparse.py'))[0],
            FileExists())

    def test_pull_with_pip_requirements_list(self):
        project_dir = 'pip-requirements-list'
        self.run_snapcraft('build', project_dir)
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python2', 'install', 'lib',
                'python2*', 'site-packages', 'argparse.py'))[0],
            FileExists())
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python2', 'install', 'lib',
                'python2*', 'site-packages', 'jsonschema'))[0],
            DirExists())
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'lib',
                'python3*', 'site-packages', 'argparse.py'))[0],
            FileExists())
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'lib',
                'python3*', 'site-packages', 'jsonschema'))[0],
            DirExists())

    def test_build_rewrites_shebangs(self):
        """Verify that LP: #1597919 doesn't come back."""

        project_dir = 'python-entry-point'
        self.run_snapcraft('stage', project_dir)
        python2_entry_point = os.path.join(
            project_dir, 'stage', 'bin', 'python2_test')
        python3_entry_point = os.path.join(
            project_dir, 'stage', 'bin', 'python3_test')
        python_entry_point = os.path.join(
            project_dir, 'stage', 'bin', 'python_test')

        with open(python2_entry_point) as f:
            python2_shebang = f.readline().strip()
        with open(python3_entry_point) as f:
            python3_shebang = f.readline().strip()
        with open(python_entry_point) as f:
            python_shebang = f.readline().strip()

        self.assertEqual('#!/usr/bin/env python', python2_shebang)
        self.assertEqual('#!/usr/bin/env python3', python3_shebang)
        self.assertEqual('#!/usr/bin/env python3', python_shebang)

    def test_build_does_not_keep_pyc_or_pth_files_in_install(self):
        # .pyc and .pyc files collide between parts.
        # There is no way to tell pip or setup.py to disable generation of
        # .pyc
        # The .pth files are only relevant if found inside the pre compiled
        # site-packges directory so we don't want those either.
        project_dir = 'pip-requirements-file'
        self.run_snapcraft('stage', project_dir)

        pyc_files = []
        pth_files = []
        for _, _, files in os.walk(os.path.join(project_dir, 'stage')):
            pyc_files.extend([f for f in files if f.endswith('pyc')])
            pth_files.extend([f for f in files if f.endswith('pth')])

        self.assertEqual([], pyc_files)
        self.assertEqual([], pth_files)

    def test_build_doesnt_get_bad_install_directory_lp1586546(self):
        """Verify that LP: #1586546 doesn't come back."""
        project_dir = 'python-pyyaml'
        self.run_snapcraft('stage', project_dir)
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python2', 'install', 'lib',
                'python2*', 'site-packages', 'yaml'))[0],
            DirExists())
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'lib',
                'python3*', 'site-packages', 'yaml'))[0],
            DirExists())

    def test_pypi_package_dep_satisfied_by_stage_package(self):
        """yamllint depends on yaml which is a stage-package."""
        project_dir = 'python-with-stage-packages'
        self.run_snapcraft('stage', project_dir)
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python2', 'install', 'lib',
                'python2*', 'site-packages', 'yamllint'))[0],
            DirExists())
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python2', 'install', 'usr', 'lib',
                'python2*', 'dist-packages', 'yaml'))[0],
            DirExists())
        self.assertEqual(
            glob(os.path.join(
                project_dir, 'parts', 'python2', 'install', 'lib',
                'python2*', 'site-packages', 'yaml')),
            [])

        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'lib',
                'python3*', 'site-packages', 'yamllint'))[0],
            DirExists())
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'usr', 'lib',
                'python3*', 'dist-packages', 'yaml'))[0],
            DirExists())
        self.assertEqual(
            glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'lib',
                'python3*', 'site-packages', 'yaml')),
            [])

    def test_pull_a_package_from_bzr(self):
        project_dir = 'pip-bzr'
        self.run_snapcraft('pull', project_dir)
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'pip-bzr', 'packages',
                'curtin-*.zip'))[0],
            FileExists())

    def test_build_with_data_files_with_root(self):
        project_dir = 'pip-root-data-files'
        self.run_snapcraft('build', project_dir)
        self.assertThat(
            glob(os.path.join(
                project_dir, 'parts', 'root', 'install',
                'lib', 'python3*', 'site-packages', 'etc', 'broken.txt'))[0],
            FileExists())
