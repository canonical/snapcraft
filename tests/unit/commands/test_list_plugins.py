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
from unittest import mock

from testtools.matchers import Equals, Contains

from tests import fixture_setup
from . import CommandBaseTestCase


class ListPluginsCommandTestCase(CommandBaseTestCase):

    scenarios = [
        ("list-plugins", {"command_name": "list-plugins"}),
        ("plugins alias", {"command_name": "plugins"}),
    ]

    default_plugin_output = (
        "fake      dotruby         kernelz  checkbox-provider  oxide  hornet      dumper  "
        "jamesbuild  none   macaroon    gmake\nnot-real  godependencies  planton  "
        "snake              web    fake-tools  cradle  maker       pmake  breaktools\n"
    )

    # fake plugin list, made up names so it is obvious when it fails that
    # we are not using real data, but use names to have some sort of simulation
    # with reality
    plugin_list = [
        "fake",
        "not-real",
        "dotruby",
        "godependencies",
        "kernelz",
        "planton",
        "checkbox-provider",
        "snake",
        "oxide",
        "web",
        "hornet",
        "fake-tools",
        "dumper",
        "cradle",
        "jamesbuild",
        "maker",
        "none",
        "pmake",
        "macaroon",
        "breaktools",
        "gmake",
    ]

    def setUp(self):
        super().setUp()

        self.maxDiff = None

        # Generate fake plugin data
        iter_modules_result = (("", p, "") for p in self.plugin_list)

        patcher = mock.patch("pkgutil.iter_modules")
        modules_mock = patcher.start()
        modules_mock.return_value = iter_modules_result
        self.addCleanup(patcher.stop)

    def test_list_plugins_non_tty(self):
        fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(fake_terminal)

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(self.default_plugin_output))

    def test_list_plugins_large_terminal(self):
        fake_terminal = fixture_setup.FakeTerminal(columns=999)
        self.useFixture(fake_terminal)

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Equals(self.default_plugin_output))

    def test_list_plugins_small_terminal(self):
        fake_terminal = fixture_setup.FakeTerminal(columns=60)
        self.useFixture(fake_terminal)

        expected_output = [
            "fake            checkbox-provider  dumper      macaroon",
            "not-real        snake              cradle      breaktools",
            "dotruby         oxide              jamesbuild  gmake",
            "godependencies  web                maker",
            "kernelz         hornet             none",
            "planton         fake-tools         pmake",
        ]

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        output_slice = [o.strip() for o in result.output.splitlines()]
        self.assertThat(output_slice, Equals(expected_output))
