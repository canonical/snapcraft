# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016, 2017 Canonical Ltd
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
        self.run_snapcraft('stage', 'simple-copy')

        self.assertThat(
            os.path.join('stage', 'dst'),
            FileContains('I got copied\n'))
        self.assertThat(
            os.path.join('stage', 'dstdir', 'srcdirfile.txt'),
            FileContains('A file in the source directory\n'))

    def test_copy_plugin_with_source(self):
        self.run_snapcraft('stage', 'copy-with-source')

        self.assertThat(
            os.path.join('stage', 'file'),
            FileContains('A file\n'))
        self.assertThat(
            os.path.join('stage', 'directory', 'file'),
            FileContains('A file in directory\n'))
