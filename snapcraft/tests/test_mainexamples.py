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
import io
import logging
import os
import pkg_resources
import shutil
import sys
from tempfile import mktemp, TemporaryDirectory, NamedTemporaryFile
from unittest import mock

import snapcraft.mainexamples

from snapcraft.tests import TestCase


class TestMain(TestCase):

    @mock.patch('snapcraft.internal.log.configure')
    def test_command_error(self, mock_log_configure):
        with mock.patch('snapcraft.mainexamples.copy') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            with self.assertRaises(SystemExit) as cm:
                snapcraft.mainexamples.main(['/tmp/foo'])

        self.assertEqual(str(cm.exception), 'some error')
        mock_log_configure.assert_called_once_with(log_level=logging.INFO)

    @mock.patch('pkg_resources.require')
    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_devel_version(self, mock_stdout, mock_resources):
        mock_resources.side_effect = pkg_resources.DistributionNotFound()
        sys.argv = ['/usr/bin/snapcraft-examples', '--version']

        with self.assertRaises(SystemExit):
            snapcraft.mainexamples.main()

        self.assertEqual(mock_stdout.getvalue(), 'devel\n')

    def test_providing_non_existing_path(self):
        sys.argv = ['/usr/bin/snapcraft-examples', "/tmp/foo"]

        with mock.patch('snapcraft.mainexamples.copy') as mock_cmd:
            snapcraft.mainexamples.main()
            mock_cmd.assert_called_once_with("/tmp/foo")

    def test_asking_for_path(self):
        sys.argv = ['/usr/bin/snapcraft-examples']
        with mock.patch('builtins.input') as input_mock:
            input_mock.return_value = "/tmp/foo"
            with mock.patch('snapcraft.mainexamples.copy') as mock_copy:
                snapcraft.mainexamples.main()
                mock_copy.assert_called_once_with("/tmp/foo")

    def test_always_return_abs_path(self):
        sys.argv = ['/usr/bin/snapcraft-examples']
        with mock.patch('builtins.input') as input_mock:
            input_mock.return_value = "foo"
            with mock.patch('snapcraft.mainexamples.copy') as mock_copy:
                snapcraft.mainexamples.main()
                mock_copy.assert_called_once_with(os.path.abspath("foo"))


class TestCopy(TestCase):

    def setUp(self):
        super().setUp()
        self.temp_dir = None
        self.first_example_name = "01-hello-world-cli"

    def tearDown(self):
        with suppress(OSError):
            if self.temp_dir:
                shutil.rmtree(self.temp_dir)
        super().tearDown()

    @mock.patch('snapcraft.mainexamples.get_examplesdir')
    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_copy_example_unexisting_path(self, mock_stdout,
                                          mock_get_examplesdir):
        mock_get_examplesdir.return_value = \
            os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
        self.temp_dir = mktemp()
        snapcraft.mainexamples.copy(self.temp_dir)

        self.assertTrue(os.path.isdir(self.temp_dir), "dest path exists")
        self.assertIn(self.first_example_name, os.listdir(self.temp_dir),
                      "first example present")
        self.assertIn(os.path.join(self.temp_dir, self.first_example_name),
                      mock_stdout.getvalue())

    @mock.patch('snapcraft.mainexamples.get_examplesdir')
    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_copy_example_existing_path(self, mock_stdout,
                                        mock_get_examplesdir):
        mock_get_examplesdir.return_value = \
            os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
        with TemporaryDirectory() as temp_dir:
            snapcraft.mainexamples.copy(temp_dir)

            # we install in a subdirectory
            dest_path = os.path.join(temp_dir, "snapcraft-examples")

            self.assertTrue(os.path.isdir(dest_path), "dest path exists: {}")
            self.assertIn(self.first_example_name, os.listdir(dest_path),
                          "first example present")
            self.assertIn(os.path.join(dest_path, self.first_example_name),
                          mock_stdout.getvalue())

    @mock.patch('snapcraft.mainexamples.get_examplesdir')
    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_copy_example_on_existing_file(self, mock_stdout,
                                           mock_get_examplesdir):
        mock_get_examplesdir.return_value = \
            os.path.join(os.path.dirname(__file__), '..', '..', 'examples')
        with NamedTemporaryFile() as temp_file:
            self.assertRaises(NotADirectoryError, snapcraft.mainexamples.copy,
                              temp_file.name)
