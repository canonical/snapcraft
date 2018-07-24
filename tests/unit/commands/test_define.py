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

from testtools.matchers import Contains, Equals

import snapcraft.internal.errors
from tests.unit import TestWithFakeRemoteParts
from . import CommandBaseTestCase


class DefineCommandTestCase(CommandBaseTestCase, TestWithFakeRemoteParts):
    def test_defining_a_part_that_exists(self):
        result = self.run_command(["define", "curl"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Maintainer: 'none'
            Description: test entry for curl

            curl:
              plugin: autotools
              source: http://curl.org"""
                )
            ),
        )

    def test_defining_unexisting_part_raises_exception(self):
        raised = self.assertRaises(
            snapcraft.internal.errors.PartNotInCacheError,
            self.run_command,
            ["define", "curler"],
        )

        self.assertThat(raised.part_name, Equals("curler"))

    def test_defining_a_part_with_multiline_description(self):
        result = self.run_command(["define", "multiline-part"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Maintainer: 'none'
            Description: this is a multiline description
            this is a multiline description
            this is a multiline description


            multiline-part:
              plugin: go
              source: http://source.tar.gz"""
                )
            ),
        )
