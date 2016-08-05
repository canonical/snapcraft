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

import snapcraft
from snapcraft.plugins.dump import DumpPlugin
from snapcraft.tests import TestCase


class DumpPluginTestCase(TestCase):

    def setUp(self):
        super().setUp()
        self.project_options = snapcraft.ProjectOptions()

    def test_dumping_nothing(self):
        class Options:
            source = '.'

        plugin = DumpPlugin('dump', Options(), self.project_options)
        plugin.pull()
        plugin.build()

        self.assertEqual(os.listdir(plugin.installdir), [])

    def test_dumping_with_contents(self):
        class Options:
            source = '.'

        open('file1', 'w').close()
        open('file2', 'w').close()
        os.mkdir('dir1')
        open(os.path.join('dir1', 'subfile1'), 'w').close()

        plugin = DumpPlugin('dump', Options(), self.project_options)
        plugin.pull()
        plugin.build()

        contents = os.listdir(plugin.installdir)
        contents.sort()
        self.assertEqual(contents, ['dir1', 'file1', 'file2'])
        self.assertEqual(os.listdir(os.path.join(plugin.installdir, 'dir1')),
                         ['subfile1'])
