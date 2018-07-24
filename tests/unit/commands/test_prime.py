# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
import snapcraft.internal.errors

import fixtures
from testtools.matchers import Equals, DirExists, Not

from . import LifecycleCommandsBaseTestCase


class PrimeCommandTestCase(LifecycleCommandsBaseTestCase):
    def test_prime_invalid_part(self):
        self.make_snapcraft_yaml("prime")

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command,
            ["prime", "no-prime"],
        )

        self.assertThat(
            str(raised),
            Equals("The part named 'no-prime' is not defined in 'snap/snapcraft.yaml'"),
        )

    def test_prime_defaults(self):
        parts = self.make_snapcraft_yaml("prime")

        result = self.run_command(["prime"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(parts[0]["part_dir"], DirExists())

        self.verify_state("prime0", parts[0]["state_dir"], "prime")

    def test_prime_one_part_only_from_3(self):
        parts = self.make_snapcraft_yaml("prime", n=3)

        result = self.run_command(["prime", "prime1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(parts[1]["part_dir"], DirExists())

        self.verify_state("prime1", parts[1]["state_dir"], "prime")

        for i in [0, 2]:
            self.assertThat(parts[i]["part_dir"], Not(DirExists()))
            self.assertThat(parts[i]["state_dir"], Not(DirExists()))

    def test_prime_ran_twice_is_a_noop(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        parts = self.make_snapcraft_yaml("prime")

        result = self.run_command(["prime"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            fake_logger.output,
            Equals(
                "Pulling prime0 \n"
                "Building prime0 \n"
                "Staging prime0 \n"
                "Priming prime0 \n"
            ),
        )

        self.assertThat(self.prime_dir, DirExists())
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(parts[0]["part_dir"], DirExists())

        self.verify_state("prime0", parts[0]["state_dir"], "prime")

        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        result = self.run_command(["prime"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            fake_logger.output,
            Equals(
                "Skipping pull prime0 (already ran)\n"
                "Skipping build prime0 (already ran)\n"
                "Skipping stage prime0 (already ran)\n"
                "Skipping prime prime0 (already ran)\n"
                "The requested action has already been taken. Consider\n"
                "specifying parts, or clean the steps you want to run "
                "again.\n"
            ),
        )
