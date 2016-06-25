# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
from unittest import mock

from snapcraft.main import main
from snapcraft import tests


class FakeStdout(io.StringIO):
    """A fake stdout using StringIO implementing the missing fileno attrib."""

    def fileno(self):
        return 1


class FakeTerminalSize:

    def __init__(self, columns=80):
        self.columns = columns


class ListPluginsCommandTestCase(tests.TestCase):

    # plugin list when wrapper at MAX_CHARACTERS_WRAP
    default_plugin_output = (
        'ant        catkin  copy  gulp  kbuild  make   nil     python2  '
        'qmake  tar-content\n'
        'autotools  cmake   go    jdk   kernel  maven  nodejs  python3  '
        'scons\n')

    def setUp(self):
        super().setUp()

        patcher = mock.patch('os.isatty')
        self.mock_isatty = patcher.start()
        self.mock_isatty.return_value = True
        self.addCleanup(patcher.stop)

        patcher = mock.patch('shutil.get_terminal_size')
        self.mock_terminal_size = patcher.start()
        self.mock_terminal_size.return_value = FakeTerminalSize()
        self.addCleanup(patcher.stop)

    @mock.patch('sys.stdout', new_callable=FakeStdout)
    def test_list_plugins_non_tty(self, mock_stdout):
        self.mock_isatty.return_value = False
        main(['list-plugins'])
        self.assertEqual(mock_stdout.getvalue(), self.default_plugin_output)

    @mock.patch('sys.stdout', new_callable=FakeStdout)
    def test_list_plugins_large_terminal(self, mock_stdout):
        self.mock_terminal_size.return_value = FakeTerminalSize(999)
        main(['list-plugins'])
        self.assertEqual(mock_stdout.getvalue(), self.default_plugin_output)

    @mock.patch('sys.stdout', new_callable=FakeStdout)
    def test_list_plugins_small_terminal(self, mock_stdout):
        self.mock_terminal_size.return_value = FakeTerminalSize(60)
        expected_output = (
            'ant        copy  kbuild  nil      qmake      \n'
            'autotools  go    kernel  nodejs   scons      \n'
            'catkin     gulp  make    python2  tar-content\n'
            'cmake      jdk   maven   python3\n')
        main(['list-plugins'])
        self.assertEqual(mock_stdout.getvalue(), expected_output)
