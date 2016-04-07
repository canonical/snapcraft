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

import io
import mock
import logging
import pkg_resources
import sys

import snapcraft.main

from snapcraft.tests import TestCase


class TestMain(TestCase):

    def test_default_command_is_snap(self):
        with mock.patch('snapcraft.lifecycle.snap') as mock_cmd:
            snapcraft.main.main([])
            mock_cmd.assert_called_once_with(None, None)

    @mock.patch('snapcraft.log.configure')
    def test_command_error(self, mock_log_configure):
        with mock.patch('snapcraft.topic_help') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            with self.assertRaises(SystemExit) as cm:
                snapcraft.main.main(['help', 'topics'])

        self.assertEqual(str(cm.exception), 'some error')
        mock_log_configure.assert_called_once_with(log_level=logging.INFO)

    @mock.patch('snapcraft.log.configure')
    def test_command_error_debug(self, mock_log_configure):
        with mock.patch('snapcraft.topic_help') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            # When verbose, the exception should be re-raised instead of
            # SystemExit. Note that this test works since SystemExit doesn't
            # inherit from Exception.
            with self.assertRaises(Exception) as cm:
                snapcraft.main.main(['help', 'topics', '--debug'])

        self.assertEqual(str(cm.exception), 'some error')
        mock_log_configure.assert_called_once_with(
            log_level=logging.DEBUG)

    def test_command_parallel_build(self):
        self.assertTrue(snapcraft.common.get_enable_parallel_builds())

        with mock.patch('snapcraft.topic_help'):
            snapcraft.main.main(['help', 'topics'])

        self.assertTrue(snapcraft.common.get_enable_parallel_builds())

    def test_command_disable_parallel_build(self):
        self.assertTrue(snapcraft.common.get_enable_parallel_builds())

        with mock.patch('snapcraft.lifecycle.execute'):
            snapcraft.main.main(['--no-parallel-build', 'build'])

        self.assertFalse(snapcraft.common.get_enable_parallel_builds())

    @mock.patch('pkg_resources.require')
    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_devel_version(self, mock_stdout, mock_resources):
        mock_resources.side_effect = pkg_resources.DistributionNotFound()
        sys.argv = ['/usr/bin/snapcraft', '--version']

        with self.assertRaises(SystemExit):
            snapcraft.main.main()

        self.assertEqual(mock_stdout.getvalue(), 'devel\n')
