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
    EndsWith,
    DirExists,
    FileExists
)

import integration_tests


class PythonPluginTestCase(integration_tests.TestCase):

    def test_snap_with_pypi_config(self):
        project_dir = 'pypi-config'
        self.run_snapcraft('snap', project_dir)

        expected_files = (
            os.path.join('stage', 'usr', 'bin', 'config.py'),
            os.path.join('stage', 'usr', 'bin', 'python3'),
            os.path.join('snap', 'usr', 'bin', 'config.py'),
            os.path.join('snap', 'usr', 'bin', 'python3'),
            os.path.join('snap', 'meta', 'hooks', 'config'),
        )
        for expected_file in expected_files:
            self.assertThat(
                os.path.join(project_dir, expected_file),
                FileExists())

        expected_config = (
            'exec "$SNAP_APP_PATH/usr/bin/python3" '
            '"$SNAP_APP_PATH/usr/bin/config.py" $*\n')
        with open(os.path.join(
                project_dir, 'snap', 'meta', 'hooks', 'config')) as config:
            config_contents = config.read()
        self.assertThat(config_contents, EndsWith(expected_config))

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
