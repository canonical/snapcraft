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

from testtools.matchers import FileContains

import integration_tests


class EnvironmentTestCase(integration_tests.TestCase):

    def test_environment(self):
        project_dir = 'snapcraft-environment'
        self.run_snapcraft('stage', project_dir)

        abs_project_dir = os.path.join(os.path.abspath('.'), project_dir)
        stage_dir = os.path.join(abs_project_dir, 'stage')
        part_install_dir = os.path.join(abs_project_dir,
                                        'parts', 'env', 'install')

        test_name = os.path.join(stage_dir, 'test_name')
        test_version = os.path.join(stage_dir, 'test_version')
        test_stage = os.path.join(stage_dir, 'test_stage')
        test_part_install = os.path.join(stage_dir, 'test_part_install')

        self.assertThat(test_name, FileContains('test-environment'))
        self.assertThat(test_version, FileContains('0.1'))
        self.assertThat(test_stage, FileContains(stage_dir))
        self.assertThat(test_part_install, FileContains(part_install_dir))
