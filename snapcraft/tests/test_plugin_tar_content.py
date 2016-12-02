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

import os.path

import snapcraft
from snapcraft.plugins.tar_content import TarContentPlugin
from snapcraft.tests import TestCase


class TestTarContentPlugin(TestCase):

    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions()

        # setup the expected target dir in our tempdir
        self.build_prefix = 'parts/tar_content/build/'
        os.makedirs(self.build_prefix)
        self.install_prefix = 'parts/tar_content/install/'
        os.makedirs(self.install_prefix)

    def test_dest_abs_path_raises_exception(self):
        class Options:
            source = '.'
            destination = '/destdir1'
        # ensure that a absolute path for a destination directory
        # raises an exception
        with self.assertRaises(ValueError) as raised:
            TarContentPlugin('tar_content', Options(),
                             self.project_options)

        self.assertEqual(raised.exception.__str__(),
                         "path '/destdir1' must be relative")

    def test_install_destination_dir_exists(self):
        class Options:
            destination = 'destdir1'

        t = TarContentPlugin('tar_content', Options(),
                             self.project_options)
        open(os.path.join(t.build_basedir, 'test.txt'), 'w').close()
        t.build()

        self.assertTrue(
            os.path.exists(os.path.join(self.install_prefix, 'destdir1')))
        self.assertTrue(
            os.path.exists(
                os.path.join(self.install_prefix, 'destdir1/test.txt')))

    def test_without_destination_dir_attribute_defined(self):
        class Options:
            source = '.'
            destination = None
        TarContentPlugin('tar_content', Options(),
                         self.project_options)

        self.assertTrue(
            os.path.exists(os.path.join(self.build_prefix)))
