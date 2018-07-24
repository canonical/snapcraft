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

import fixtures
import os

from click.testing import CliRunner

from snapcraft.cli.snapcraftctl._runner import run
from tests import unit


class CommandBaseNoFifoTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.runner = CliRunner()

    def run_command(self, args, **kwargs):
        return self.runner.invoke(run, args, catch_exceptions=False, **kwargs)


class CommandBaseTestCase(CommandBaseNoFifoTestCase):
    def setUp(self):
        super().setUp()

        tempdir = self.useFixture(fixtures.TempDir()).path
        self.call_fifo = os.path.join(tempdir, "call_fifo")
        self.feedback_fifo = os.path.join(tempdir, "feedback_fifo")

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFTCTL_CALL_FIFO", self.call_fifo)
        )
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFTCTL_FEEDBACK_FIFO", self.feedback_fifo
            )
        )

        # Create the FIFOs
        open(self.call_fifo, "w").close()
        open(self.feedback_fifo, "w").close()
