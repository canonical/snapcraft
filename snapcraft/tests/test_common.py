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

from snapcraft import (
    common,
    tests
)


class CommonTestCase(tests.TestCase):

    def test_get_stagedir(self):
        self.assertEqual(
            os.path.join(self.path, 'stage'), common.get_stagedir())

    def test_get_snapdir(self):
        self.assertEqual(
            os.path.join(self.path, 'snap'), common.get_snapdir())

    def test_get_default_plugindir(self):
        self.assertEqual(
            '/usr/share/snapcraft/plugins', common.get_plugindir())

    def test_set_plugindir(self):
        plugindir = os.path.join(self.path, 'testplugin')
        common.set_plugindir(plugindir)
        self.assertEqual(plugindir, common.get_plugindir())

    def test_isurl(self):
        self.assertTrue(common.isurl('git://'))
        self.assertTrue(common.isurl('bzr://'))
        self.assertFalse(common.isurl('./'))
        self.assertFalse(common.isurl('/foo'))
        self.assertFalse(common.isurl('/fo:o'))
