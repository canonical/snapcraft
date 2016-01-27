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

import mock

import snapcraft.main

from snapcraft.tests import TestCase


class TestMain(TestCase):

    def setUp(self):
        super(TestMain, self).setUp()

        patcher = mock.patch('snapcraft.main.docopt')
        self.mock_docopt = patcher.start()
        self.addCleanup(patcher.stop)

    def test_valid_commands(self):
        expected = [
            'list-parts',
            'list-plugins',
            'init',
            'add-part',
            'pull',
            'build',
            'clean',
            'stage',
            'strip',
            'snap',
            'help',
            'login',
            'logout',
            'upload',
        ]
        self.assertEqual(snapcraft.main._VALID_COMMANDS, expected)

    def test_invalid_command(self):
        self.mock_docopt.return_value = {
            'COMMAND': 'invalid',
            'ARGS': [],
        }

        with self.assertRaises(SystemExit) as cm:
            snapcraft.main.main()
        self.assertEqual(
            str(cm.exception), "Command 'invalid' was not recognized")

    def test_command_error(self):
        self.mock_docopt.return_value = {
            'COMMAND': 'help',
            'ARGS': [],
        }
        with mock.patch('snapcraft.commands.help.main') as mock_cmd:
            mock_cmd.side_effect = Exception('some error')

            with self.assertRaises(SystemExit) as cm:
                snapcraft.main.main()

        self.assertEqual(str(cm.exception), 'some error')
