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
import snapcraft.common

from snapcraft.tests import TestCase


class TestMain(TestCase):

    def test_valid_commands(self):
        expected = [
            'list-parts',
            'list-plugins',
            'init',
            'add-part',
            'pull',
            'build',
            'clean',
            'cleanbuild',
            'stage',
            'strip',
            'snap',
            'help',
            'login',
            'logout',
            'upload',
        ]
        self.assertEqual(snapcraft.main._VALID_COMMANDS, expected)

    @mock.patch('snapcraft.main.docopt')
    def test_invalid_command(self, mock_docopt):
        mock_docopt.return_value = {
            'COMMAND': 'invalid',
            '--debug': False,
            '--no-parallel-build': False,
            '--enable-geoip': False,
            '--target-arch': None,
            'ARGS': [],
        }

        with self.assertRaises(SystemExit) as cm:
            snapcraft.main.main()
        self.assertEqual(
            str(cm.exception), "Command 'invalid' was not recognized")

    @mock.patch('snapcraft.main.docopt')
    def test_default_command_is_snap(self, mock_docopt):
        mock_docopt.return_value = {
            'COMMAND': '',
            '--debug': False,
            '--no-parallel-build': False,
            '--enable-geoip': False,
            '--target-arch': None,
            'ARGS': [],
        }
        with mock.patch('snapcraft.commands.snap.main') as mock_cmd:
            snapcraft.main.main()
            mock_cmd.assert_called_once_with(argv=[])

    @mock.patch('snapcraft.log.configure')
    @mock.patch('snapcraft.main.docopt')
    def test_command_error(self, mock_docopt, mock_log_configure):
        mock_docopt.return_value = {
            'COMMAND': 'help',
            '--debug': False,
            '--no-parallel-build': False,
            '--enable-geoip': False,
            '--target-arch': None,
            'ARGS': [],
        }

        with mock.patch('snapcraft.commands.help.main') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            with self.assertRaises(SystemExit) as cm:
                snapcraft.main.main()

        self.assertEqual(str(cm.exception), 'some error')
        mock_log_configure.assert_called_once_with(log_level=logging.INFO)

    @mock.patch('snapcraft.log.configure')
    @mock.patch('snapcraft.main.docopt')
    def test_command_error_debug(self, mock_docopt, mock_log_configure):
        mock_docopt.return_value = {
            'COMMAND': 'help',
            '--debug': True,
            '--no-parallel-build': False,
            '--enable-geoip': False,
            '--target-arch': None,
            'ARGS': [],
        }

        with mock.patch('snapcraft.commands.help.main') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            # When verbose, the exception should be re-raised instead of
            # SystemExit. Note that this test works since SystemExit doesn't
            # inherit from Exception.
            with self.assertRaises(Exception) as cm:
                snapcraft.main.main()

        self.assertEqual(str(cm.exception), 'some error')
        mock_log_configure.assert_called_once_with(
            log_level=logging.DEBUG)

    @mock.patch('snapcraft.main.docopt')
    def test_command_parallel_build(self, mock_docopt):
        mock_docopt.return_value = {
            'COMMAND': 'help',
            '--debug': False,
            '--no-parallel-build': False,
            '--enable-geoip': False,
            '--target-arch': None,
            'ARGS': [],
        }

        self.assertTrue(snapcraft.common.get_enable_parallel_builds())

        with mock.patch('snapcraft.commands.help.main'):
            snapcraft.main.main()

        self.assertTrue(snapcraft.common.get_enable_parallel_builds())

    @mock.patch('snapcraft.main.docopt')
    def test_command_disable_parallel_build(self, mock_docopt):
        mock_docopt.return_value = {
            'COMMAND': 'help',
            '--debug': False,
            '--no-parallel-build': True,
            '--enable-geoip': False,
            '--target-arch': None,
            'ARGS': [],
        }

        self.assertTrue(snapcraft.common.get_enable_parallel_builds())

        with mock.patch('snapcraft.commands.help.main'):
            snapcraft.main.main()

        self.assertFalse(snapcraft.common.get_enable_parallel_builds())

    @mock.patch('pkg_resources.require')
    @mock.patch('sys.stdout', new_callable=io.StringIO)
    def test_devel_version(self, mock_stdout, mock_resources):
        mock_resources.side_effect = pkg_resources.DistributionNotFound()
        sys.argv = ['/usr/bin/snapcraft', '--version']

        with self.assertRaises(SystemExit):
            snapcraft.main.main()

        self.assertEqual(mock_stdout.getvalue(), 'devel\n')
