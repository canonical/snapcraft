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
import subprocess
from unittest import mock

from snapcraft.main import main
from snapcraft import tests


class ListPluginsCommandTestCase(tests.TestCase):

    # plugin list when wrapper at MAX_CHARACTERS_WRAP
    default_plugin_output = (
        'ant        catkin  copy  jdk     kernel  maven  nodejs   python3  '
        'scons      \n'
        'autotools  cmake   go    kbuild  make    nil    python2  qmake    '
        'tar-content\n')

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    @mock.patch('subprocess.check_output')
    def test_list_plugins_large_terminal(self, mock_subprocess, mock_stdout):
        mock_subprocess.return_value = "999"
        main(['list-plugins'])
        self.assertEqual(mock_stdout.getvalue(), self.default_plugin_output)

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    @mock.patch('subprocess.check_output')
    def test_list_plugins_small_terminal(self, mock_subprocess, mock_stdout):
        mock_subprocess.return_value = "60"
        expected_output = (
            'ant        cmake  jdk     make   nodejs   qmake      \n'
            'autotools  copy   kbuild  maven  python2  scons      \n'
            'catkin     go     kernel  nil    python3  tar-content\n')
        main(['list-plugins'])
        self.assertEqual(mock_stdout.getvalue(), expected_output)

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    @mock.patch('subprocess.check_output')
    def test_list_plugins_error_invalid_terminal_size(self, mock_subprocess,
                                                      mock_stdout):
        def raise_error(cmd, stderr):
            raise OSError()
        mock_subprocess.side_effect = raise_error
        main(['list-plugins'])
        self.assertEqual(mock_stdout.getvalue(), self.default_plugin_output)

    @mock.patch('sys.stdout', new_callable=io.StringIO)
    @mock.patch('subprocess.check_output')
    def test_list_plugins_error_invalid_subprocess_call(self, mock_subprocess,
                                                        mock_stdout):
        def raise_error(cmd, stderr):
            raise subprocess.CalledProcessError(returncode=1, cmd=cmd)
        mock_subprocess.side_effect = raise_error
        main(['list-plugins'])
        self.assertEqual(mock_stdout.getvalue(), self.default_plugin_output)
