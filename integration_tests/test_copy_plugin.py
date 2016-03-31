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

from testtools.matchers import FileContains

import integration_tests


class CopyPluginTestCase(integration_tests.TestCase):

    def test_stage_copy_plugin(self):
        project_dir = 'simple-copy'
        self.run_snapcraft('stage', project_dir)

        self.assertThat(
            os.path.join(project_dir, 'stage', 'dst'),
            FileContains('I got copied\n'))
        self.assertThat(
            os.path.join(project_dir, 'stage', 'dstdir', 'srcdirfile.txt'),
            FileContains('A file in the source directory\n'))

    def test_copy_plugin_with_source(self):
        project_dir = 'copy-with-source'
        self.run_snapcraft('stage', project_dir)

        self.assertThat(
            os.path.join(project_dir, 'stage', 'file'),
            FileContains('A file\n'))
        self.assertThat(
            os.path.join(project_dir, 'stage', 'directory', 'file'),
            FileContains('A file in directory\n'))
