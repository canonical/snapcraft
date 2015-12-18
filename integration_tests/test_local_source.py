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

from testtools.matchers import FileExists

import integration_tests


class LocalSourceTestCase(integration_tests.TestCase):

    def test_build_local_source(self):
        project_dir = 'local-source'
        self.run_snapcraft('build', project_dir)

        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'make-project', 'build', 'stamp-all'),
            FileExists())

    def test_stage_local_source(self):
        project_dir = 'local-source'
        self.run_snapcraft('stage', project_dir)

        self.assertThat(
            os.path.join(
                project_dir, 'parts', 'make-project', 'build',
                'stamp-install'),
            FileExists())
