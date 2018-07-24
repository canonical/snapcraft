# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
from testtools.matchers import Equals, Contains

from tests import fixture_setup
from . import CommandBaseTestCase


class ListPluginsCommandTestCase(CommandBaseTestCase):

    scenarios = [
        ("list-plugins", {"command_name": "list-plugins"}),
        ("plugins alias", {"command_name": "plugins"}),
    ]

    # plugin list when wrapper at MAX_CHARACTERS_WRAP
    default_plugin_output = (
        "ament      catkin        copy    go      gulp     kbuild  maven  "
        "nodejs             python2  ruby   tar-content\n"
        "ant        catkin-tools  dotnet  godeps  jdk      kernel  meson  "
        "plainbox-provider  python3  rust   waf        \n"
        "autotools  cmake         dump    gradle  jhbuild  make    nil    "
        "python             qmake    scons\n"
    )

    def test_list_plugins_non_tty(self):
        self.maxDiff = None
        fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(fake_terminal)

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(self.default_plugin_output))

    def test_list_plugins_large_terminal(self):
        self.maxDiff = None
        fake_terminal = fixture_setup.FakeTerminal(columns=999)
        self.useFixture(fake_terminal)

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Equals(self.default_plugin_output))

    def test_list_plugins_small_terminal(self):
        self.maxDiff = None
        fake_terminal = fixture_setup.FakeTerminal(columns=60)
        self.useFixture(fake_terminal)

        expected_output = (
            "ament         dump     kernel             python2    \n"
            "ant           go       make               python3    \n"
            "autotools     godeps   maven              qmake      \n"
            "catkin        gradle   meson              ruby       \n"
            "catkin-tools  gulp     nil                rust       \n"
            "cmake         jdk      nodejs             scons      \n"
            "copy          jhbuild  plainbox-provider  tar-content\n"
            "dotnet        kbuild   python             waf        \n"
        )

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Equals(expected_output))
