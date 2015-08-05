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
import tempfile
from unittest.mock import (
    Mock,
)

from snapcraft.plugins.ubuntu import UbuntuPlugin

from snapcraft.tests import TestCase


class TestUbuntu(TestCase):

    def test_fix_symlinks(self):
        tempdirObj = tempfile.TemporaryDirectory()
        self.addCleanup(tempdirObj.cleanup)
        tempdir = tempdirObj.name

        os.makedirs(tempdir + '/a')
        open(tempdir + '/1', mode='w').close()

        os.symlink('a', tempdir + '/rel-to-a')
        os.symlink('/a', tempdir + '/abs-to-a')
        os.symlink('/b', tempdir + '/abs-to-b')
        os.symlink('1', tempdir + '/rel-to-1')
        os.symlink('/1', tempdir + '/abs-to-1')

        options = Mock()
        options.packages = ['test']
        ubuntu = UbuntuPlugin('ubuntu', options)
        ubuntu.fix_symlinks(debdir=tempdir)

        self.assertEqual(os.readlink(tempdir + '/rel-to-a'), 'a')
        self.assertEqual(os.readlink(tempdir + '/abs-to-a'), 'a')
        self.assertEqual(os.readlink(tempdir + '/abs-to-b'), '/b')
        self.assertEqual(os.readlink(tempdir + '/rel-to-1'), '1')
        self.assertEqual(os.readlink(tempdir + '/abs-to-1'), '1')
