# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
from textwrap import dedent
import logging

import fixtures
from testtools.matchers import Contains, Equals

from tests import fixture_setup
from tests.unit import TestWithFakeRemoteParts
from . import CommandBaseTestCase


class SearchCommandTestCase(CommandBaseTestCase, TestWithFakeRemoteParts):
    def test_searching_for_a_part_that_exists(self):
        result = self.run_command(["search", "curl"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            PART NAME  DESCRIPTION
            curl       test entry for curl"""
                )
            ),
        )

    def test_empty_search_searches_all(self):
        result = self.run_command(["search"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            PART NAME            DESCRIPTION
            curl                 test entry for curl
            long-described-part  this is a repetitive description this is a repetitive de...
            multiline-part       this is a multiline description
            part1                test entry for part1"""
                )
            ),
        )  # noqa

    def test_search_trims_long_descriptions(self):
        result = self.run_command(["search", "long-described-part"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            PART NAME            DESCRIPTION
            long-described-part  this is a repetitive description this is a repetitive de...
            """
                )
            ),
        )  # noqa

    def test_search_only_first_line_of_description(self):
        result = self.run_command(["search", "mulitline-part"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            PART NAME       DESCRIPTION
            multiline-part  this is a multiline description"""
                )
            ),
        )

    def test_searching_for_a_part_that_doesnt_exist_helps_out(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        result = self.run_command(["search", "part that does not exist"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            fake_logger.output,
            Contains(
                "No matches found, try to run `snapcraft update` to refresh the "
                "remote parts cache.\n"
            ),
        )

    def test_search_on_non_tty(self):
        fake_terminal = fixture_setup.FakeTerminal(isatty=False)
        self.useFixture(fake_terminal)

        result = self.run_command(["search", "curl"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            PART NAME  DESCRIPTION
            curl       test entry for curl"""
                )
            ),
        )
