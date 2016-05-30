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

from contextlib import suppress
import os
import shutil
import sys
from tempfile import mktemp, TemporaryDirectory, NamedTemporaryFile
from unittest import mock

import snapcraft.main

from snapcraft.tests import TestCase


class TestExamplesCmd(TestCase):

    def test_call_scaffold_with_parameter(self):
        sys.argv = ['/usr/bin/snapcraft', 'examples', '/tmp/foo']

        with mock.patch('snapcraft.main._scaffold_examples') as mock_cmd:
            snapcraft.main.main()
            mock_cmd.assert_called_once_with("/tmp/foo")

    def test_call_scaffold_without_parameter(self):
        sys.argv = ['/usr/bin/snapcraft', 'examples']

        with mock.patch('snapcraft.main._scaffold_examples') as mock_cmd:
            snapcraft.main.main()
            mock_cmd.assert_called_once_with(
                snapcraft.main._SNAPCRAFT_TOUR_DIR)


class TestScaffoldExample(TestCase):

    def setUp(self):
        super().setUp()
        self.temp_dir = None

    def tearDown(self):
        with suppress(OSError):
            if self.temp_dir:
                shutil.rmtree(self.temp_dir)
        super().tearDown()

    @mock.patch('snapcraft.main.get_examplesdir')
    def test_copy_example_unexisting_path(self, mock_get_examplesdir):
        mock_get_examplesdir.return_value = \
            os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
        self.temp_dir = mktemp()
        snapcraft.main._scaffold_examples(self.temp_dir)

        self.assertTrue(os.path.isdir(self.temp_dir), "dest path exists")

    @mock.patch('snapcraft.main.get_examplesdir')
    def test_copy_example_existing_path(self, mock_get_examplesdir):
        mock_get_examplesdir.return_value = \
            os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
        with TemporaryDirectory() as temp_dir:
            snapcraft.main._scaffold_examples(temp_dir)

            # we install in a subdirectory
            dest_path = os.path.join(temp_dir, "snapcraft-tour")

            self.assertTrue(os.path.isdir(dest_path), "dest path exists: {}")

    @mock.patch('snapcraft.main.get_examplesdir')
    def test_copy_example_on_existing_file(self, mock_get_examplesdir):
        mock_get_examplesdir.return_value = \
            os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
        with NamedTemporaryFile() as temp_file:
            self.assertRaises(NotADirectoryError,
                              snapcraft.main._scaffold_examples,
                              temp_file.name)
