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
        'ant        catkin  copy  go      gradle  jdk      kbuild  make   '
        'nil     plainbox-provider  python2  qmake  scons      \n'
        'autotools  cmake   dump  godeps  gulp    jhbuild  kernel  maven  '
        'nodejs  python             python3  rust   tar-content\n'
    )

    def test_list_plugins_non_tty(self):
        self.maxDiff = None
        fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(fake_terminal)

        main(['list-plugins'])
        self.assertEqual(fake_terminal.getvalue(), self.default_plugin_output)

    def test_list_plugins_large_terminal(self):
        self.maxDiff = None
        fake_terminal = fixture_setup.FakeTerminal(columns=999)
        self.useFixture(fake_terminal)

        main(['list-plugins'])
        self.assertEqual(fake_terminal.getvalue(), self.default_plugin_output)

    def test_list_plugins_small_terminal(self):
        self.maxDiff = None
        fake_terminal = fixture_setup.FakeTerminal(columns=60)
        self.useFixture(fake_terminal)

        expected_output = (
            'ant        go       kbuild  plainbox-provider  scons      \n'
            'autotools  godeps   kernel  python             tar-content\n'
            'catkin     gradle   make    python2          \n'
            'cmake      gulp     maven   python3          \n'
            'copy       jdk      nil     qmake            \n'
            'dump       jhbuild  nodejs  rust             \n'
        )

        main(['list-plugins'])
        self.assertEqual(fake_terminal.getvalue(), expected_output)
