# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Neal Gompa
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


class RpmSourceTestCase(integration_tests.TestCase):

    def test_stage_rpm(self):
        project_dir = self.copy_project_to_tmp('simple-rpm')
        self.run_snapcraft('stage', project_dir)

        self.assertThat(
            os.path.join(project_dir, 'stage', 'bin', 'hello'),
            FileExists())
        self.assertThat(
            os.path.join(project_dir, 'stage', 'usr', 'bin', 'world'),
            FileExists())
