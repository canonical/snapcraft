# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import json

from testtools.matchers import Contains, Equals, FileExists

from snapcraft_legacy.internal import errors

from . import CommandBaseNoFifoTestCase, CommandBaseTestCase


class BuildCommandTestCase(CommandBaseTestCase):
    def test_build(self):
        self.run_command(["build"])
        self.assertThat(self.call_fifo, FileExists())

        with open(self.call_fifo, "r") as f:
            data = json.loads(f.read())

        self.assertThat(data, Contains("function"))
        self.assertThat(data, Contains("args"))

        self.assertThat(data["function"], Equals("build"))
        self.assertThat(data["args"], Equals({}))

    def test_build_error(self):
        # If there is a string in the feedback, it should be considered an
        # error
        with open(self.feedback_fifo, "w") as f:
            f.write("this is an error\n")

        assert self.run_command(["build"]).exit_code == -1


class BuildCommandWithoutFifoTestCase(CommandBaseNoFifoTestCase):
    def test_build_without_fifo(self):
        raised = self.assertRaises(
            errors.SnapcraftEnvironmentError, self.run_command, ["build"]
        )

        self.assertThat(
            str(raised),
            Contains(
                "environment variable must be defined. Note that this utility is "
                "only designed for use within a snapcraft.yaml"
            ),
        )
