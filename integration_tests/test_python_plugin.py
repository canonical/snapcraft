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

import glob
import os

from testtools.matchers import (
    DirExists,
    FileExists
)

import integration_tests


class PythonPluginTestCase(integration_tests.TestCase):

    def test_pull_with_pip_requirements_file(self):
        project_dir = 'pip-requirements-file'
        self.run_snapcraft('pull', project_dir)
        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'python2', 'install', 'usr', 'lib',
                'python2.7', 'argparse.py'),
            FileExists())
        self.assertThat(
            glob.glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'usr', 'lib',
                'python3*', 'argparse.py'))[0],
            FileExists())

    def test_pull_with_pip_requirements_list(self):
        project_dir = 'pip-requirements-list'
        self.run_snapcraft('pull', project_dir)
        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'python2', 'install', 'usr', 'lib',
                'python2.7', 'argparse.py'),
            FileExists())
        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'python2', 'install', 'usr', 'lib',
                'python2.7', 'dist-packages', 'jsonschema'),
            DirExists())
        self.assertThat(
            glob.glob(os.path.join(
                project_dir, 'parts', 'python3', 'install', 'usr', 'lib',
                'python3*', 'argparse.py'))[0],
            FileExists())
        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'python3', 'install', 'usr', 'lib',
                'python3', 'dist-packages', 'jsonschema'),
            DirExists())

    def test_build_rewrites_shebangs(self):
        """Verify that LP: #1597919 doesn't come back."""

        project_dir = 'python-entry-point'
        self.run_snapcraft('stage', project_dir)
        python2_entry_point = os.path.join(
            project_dir, 'stage', 'usr', 'bin', 'python2_test')
        python3_entry_point = os.path.join(
            project_dir, 'stage', 'usr', 'bin', 'python3_test')

        with open(python2_entry_point) as f:
            python2_shebang = f.readline().strip()
        with open(python3_entry_point) as f:
            python3_shebang = f.readline().strip()

        self.assertEqual('#!/usr/bin/env python2', python2_shebang)
        self.assertEqual('#!/usr/bin/env python3', python3_shebang)

    def test_build_doesnt_get_bad_install_directory_lp1586546(self):
        """Verify that LP: #1586546 doesn't come back."""
        project_dir = 'python-pyyaml'
        self.run_snapcraft('stage', project_dir)
        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'python2', 'install', 'usr', 'lib',
                'python2.7', 'dist-packages', 'yaml'),
            DirExists())
        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'python3', 'install', 'usr', 'lib',
                'python3', 'dist-packages', 'yaml'),
            DirExists())
