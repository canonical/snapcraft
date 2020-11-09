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

import fixtures
from testtools.matchers import Contains, Equals

import snapcraft
from tests import fixture_setup

from . import CommandBaseTestCase


class ListPluginsCommandTestCase(CommandBaseTestCase):

    command_name = "list-plugins"

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

        self.fake_iter_modules = fixtures.MockPatch(
            "pkgutil.iter_modules", return_value=iter_modules_result
        )
        self.useFixture(self.fake_iter_modules)

    def test_default_from_snapcraft_yaml(self):
        self.useFixture(
            fixture_setup.SnapcraftYaml(
                self.path,
                base="core18",
                parts={"part1": {"source": ".", "plugin": "nil"}},
            )
        )

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Contains("Displaying plugins available for 'core18")
        )
        self.fake_iter_modules.mock.assert_called_once_with(
            snapcraft.plugins.v1.__path__
        )

    def test_alias(self):
        self.command_name = "plugins"
        self.useFixture(
            fixture_setup.SnapcraftYaml(
                self.path,
                base="core18",
                parts={"part1": {"source": ".", "plugin": "nil"}},
            )
        )

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))

    def test_core20_list(self):
        result = self.run_command([self.command_name, "--base", "core20"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Contains("Displaying plugins available for 'core20")
        )

        self.fake_iter_modules.mock.assert_called_once_with(
            snapcraft.plugins.v2.__path__
        )

    def test_core2y_list(self):
        # Note that core2y is some future base, _not_ allowed to be used from cmdline
        # This tests that addition of the next base will use the latest version of plugins
        snapcraft.cli.discovery.list_plugins.callback("core2y")
        self.fake_iter_modules.mock.assert_called_once_with(
            snapcraft.plugins.v2.__path__
        )

    def test_list_plugins_non_tty(self):
        fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(fake_terminal)

        result = self.run_command([self.command_name, "--base", "core18"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(self.default_plugin_output))
        self.fake_iter_modules.mock.assert_called_once_with(
            snapcraft.plugins.v1.__path__
        )

    def test_list_plugins_large_terminal(self):
        fake_terminal = fixture_setup.FakeTerminal(columns=999)
        self.useFixture(fake_terminal)

        result = self.run_command([self.command_name, "--base", "core18"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(self.default_plugin_output))
        self.fake_iter_modules.mock.assert_called_once_with(
            snapcraft.plugins.v1.__path__
        )

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

        result = self.run_command([self.command_name, "--base", "core18"])

        self.assertThat(result.exit_code, Equals(0))
        output_slice = [o.strip() for o in result.output.splitlines()][1:]
        self.assertThat(output_slice, Equals(expected_output))
        self.fake_iter_modules.mock.assert_called_once_with(
            snapcraft.plugins.v1.__path__
        )
