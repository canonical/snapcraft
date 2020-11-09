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

import logging
import pydoc
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, StartsWith

from snapcraft.cli._runner import run
from snapcraft.cli.help import _TOPICS
from tests import fixture_setup

from . import CommandBaseTestCase


class HelpCommandBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        # pydoc pager guess can fail, for tests we want a plain pager
        # anyway
        p = mock.patch("pydoc.pager", new=pydoc.plainpager)
        p.start()
        self.addCleanup(p.stop)


class HelpCommandTestCase(HelpCommandBaseTestCase):
    def test_topic_and_plugin_not_found_exits_with_tip(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        result = self.run_command(["help", "does-not-exist"])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(
            result.output, Contains("There is no help topic, plugin or command")
        )

    def test_topic_and_plugin_adds_ellipsis_for_long_arg(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        result = self.run_command(["help", "1234567890123"])

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains("1234567890..."))

    def test_print_module_help_for_valid_plugin_default_base(self):
        result = self.run_command(["help", "nil"])

        expected = "Displaying help for the 'nil' plugin for 'core20'."
        output = result.output[: len(expected)]
        self.assertThat(
            output,
            Equals(expected),
            "The help message does not start with {!r} but with "
            "{!r} instead".format(expected, output),
        )

    def test_print_module_help_for_valid_plugin_with_base(self):
        for base in ("core", "core18", "core20"):
            result = self.run_command(["help", "nil", "--base", base])

            expected = f"Displaying help for the 'nil' plugin for {base!r}."
            output = result.output[: len(expected)]
            self.expectThat(
                output,
                Equals(expected),
                "The help message does not start with {!r} but with "
                "{!r} instead".format(expected, output),
            )

    def test_print_module_help_for_valid_plugin_snapcraft_yaml(self):
        self.useFixture(
            fixture_setup.SnapcraftYaml(
                self.path,
                base="core18",
                parts={"part1": {"source": ".", "plugin": "nil"}},
            )
        )
        result = self.run_command(["help", "python", "--base", "core18"])

        expected = (
            "Displaying help for the 'python' plugin for 'core18'.\n\n"
            "The python plugin can be used for"
        )
        output = result.output[: len(expected)]
        self.assertThat(
            output,
            Equals(expected),
            "The help message does not start with {!r} but with "
            "{!r} instead".format(expected, output),
        )

    def test_print_module_named_with_dashes_help_for_valid_plugin(self):
        result = self.run_command(["help", "plainbox-provider", "--base", "core18"])

        expected = "Displaying help for the 'plainbox-provider' plugin for 'core18'."
        self.assertThat(result.output, StartsWith(expected))

    def test_show_module_help_with_devel_for_valid_plugin(self):
        result = self.run_command(["help", "nil", "--devel"])

        expected = "Help on module snapcraft.plugins.v2.nil in snapcraft.plugins"
        output = result.output[: len(expected)]

        self.assertThat(
            output,
            Equals(expected),
            "The help message does not start with {!r} but with "
            "{!r} instead".format(expected, output),
        )

    def test_print_topics(self):
        result = self.run_command(["help", "topics"])

        output = result.output.strip().split("\n")
        for t in _TOPICS:
            self.assertTrue(
                t in output, "Missing topic: {!r} in {!r}".format(t, output)
            )

    def test_print_topic_help_for_valid_topic(self):
        result = self.run_command(["help", "sources"])

        expected = "Common 'source' options."
        output = result.output[: len(expected)]
        self.assertThat(
            output,
            Equals(expected),
            "The help message does not start with {!r} but with "
            "{!r} instead".format(expected, output),
        )

    def test_print_generic_help_by_default(self):
        result = self.run_command(["help"])

        self.assertThat(
            result.output, Contains("Snapcraft is a delightful packaging tool.")
        )
        self.assertThat(result.output, Contains("For more help"))

    def test_no_unicode_in_help_strings(self):
        helps = ["topics"]

        for key in _TOPICS.keys():
            helps.append(str(key))

        # Get a list of plugins
        import os
        from pathlib import Path

        import snapcraft.plugins

        for plugin in Path(snapcraft.plugins.__path__[0]).glob("*.py"):
            if os.path.isfile(str(plugin)) and not os.path.basename(
                str(plugin)
            ).startswith("_"):
                helps.append(os.path.basename(str(plugin)[:-3]))

        for key in helps:
            result = self.run_command(["help", key])
            # An UnicodeEncodeError will be raised if the help text has
            # non-ASCII characters.
            result.output.encode("ascii")


class TopicWithDevelTestCase(HelpCommandBaseTestCase):
    def test_print_topic_help_with_devel_for_valid_topic(self):
        expected = {
            "sources": "Help on package snapcraft",
            "plugins": "Help on package snapcraft",
        }

        for topic in _TOPICS:
            result = self.run_command(["help", topic, "--devel"])
            output = result.output[: len(expected[topic])]
            self.assertThat(
                output,
                Equals(expected[topic]),
                "The help message does not start with {!r} but with "
                "{!r} instead".format(expected[topic], output),
            )


class TestHelpForCommand(HelpCommandBaseTestCase):
    def test_help_for_command(self):
        for command in run.commands:
            result = self.run_command(["help", command])
            self.assertThat(result.exit_code, Equals(0))
            # Verify that the first line of help text is correct
            # to ensure no name squatting takes place.
            self.assertThat(
                result.output, Contains(run.commands[command].help.split("\n")[0])
            )
