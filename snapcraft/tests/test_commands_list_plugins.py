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

from snapcraft.main import main
from snapcraft import tests
from snapcraft.tests import fixture_setup


class ListPluginsCommandTestCase(tests.TestCase):

    # plugin list when wrapper at MAX_CHARACTERS_WRAP
    default_plugin_output = (
        'ant        catkin  copy  gradle  jdk     kernel  maven  nodejs '
        '            python2  qmake  script     \nautotools  cmake   go'
        '    gulp    kbuild  make    nil    plainbox-provider  python3  '
        'scons  tar-content\n')

    def test_list_plugins_non_tty(self):
        fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(fake_terminal)

        main(['list-plugins'])
        self.assertEqual(fake_terminal.getvalue(), self.default_plugin_output)

    def test_list_plugins_large_terminal(self):
        fake_terminal = fixture_setup.FakeTerminal(columns=999)
        self.useFixture(fake_terminal)

        main(['list-plugins'])
        self.assertEqual(fake_terminal.getvalue(), self.default_plugin_output)

    def test_list_plugins_small_terminal(self):
        fake_terminal = fixture_setup.FakeTerminal(columns=60)
        self.useFixture(fake_terminal)

        expected_output = (
            'ant        go      kernel  plainbox-provider  script     \n'
            'autotools  gradle  make    python2            tar-content\n'
            'catkin     gulp    maven   python3          \n'
            'cmake      jdk     nil     qmake            \n'
            'copy       kbuild  nodejs  scons            \n')

        main(['list-plugins'])
        self.assertEqual(fake_terminal.getvalue(), expected_output)
