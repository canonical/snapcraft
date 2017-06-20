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

import os

from testtools.matchers import FileExists

import integration_tests
import testscenarios


class LocalSourceTestCase(integration_tests.TestCase):

    def test_build_local_source(self):
        self.run_snapcraft('build', 'local-source')

        self.assertThat(
            os.path.join(
                self.parts_dir, 'make-project', 'build', 'stamp-all'),
            FileExists())

    def test_stage_local_source(self):
        self.run_snapcraft('stage', 'local-source')

        self.assertThat(
            os.path.join(
                self.parts_dir, 'make-project', 'build',
                'stamp-install'),
            FileExists())


class LocalSourceTypeTestCase(integration_tests.TestCase):

    def test_build_local_source(self):
        self.run_snapcraft('build', 'local-source-type')

        self.assertThat(
            os.path.join(
                self.parts_dir, 'make-project', 'build', 'stamp-all'),
            FileExists())

    def test_stage_local_source(self):
        self.run_snapcraft('stage', 'local-source')

        self.assertThat(
            os.path.join(
                self.parts_dir, 'make-project', 'build',
                'stamp-install'),
            FileExists())


class LocalSourceSubfoldersTestCase(
        testscenarios.WithScenarios, integration_tests.TestCase):

    scenarios = [
        ('Top folder',
            {'subfolder': '.'}),
        ('Sub folder Level 1',
            {'subfolder': 'packaging'}),
        ('Sub folder Level 2',
            {'subfolder': os.path.join('packaging', 'snap-package')}),
        ('Sub folder Level 3',
            {'subfolder': os.path.join(
                'packaging', 'snap-package', 'yes-really-deep')}),
    ]

    def test_pull_local_source(self):
        self.copy_project_to_cwd('local-source-subfolders')
        os.chdir(self.subfolder)
        self.run_snapcraft('pull')
