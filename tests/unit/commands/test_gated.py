# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import textwrap

from testtools.matchers import Contains, Equals

import snapcraft.storeapi.errors
from . import StoreCommandsBaseTestCase


account_info_data = {"snaps": {"16": {"core": {"snap-id": "good"}}}}


class GatedCommandTestCase(StoreCommandsBaseTestCase):
    def test_gated_unknown_snap(self):
        self.client.login("dummy", "test correct password")

        raised = self.assertRaises(
            snapcraft.storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["gated", "notfound"],
        )

        self.assertThat(str(raised), Equals("Snap 'notfound' was not found."))

    def test_gated_success(self):
        self.client.login("dummy", "test correct password")

        result = self.run_command(["gated", "core"])

        self.assertThat(result.exit_code, Equals(0))
        expected_output = textwrap.dedent(
            """\
            Name      Revision  Required    Approved
            snap-1           3  True        2016-09-19T21:07:27Z
            snap-2           5  False       2016-09-19T21:07:27Z
            snap-3           -  True        2016-09-19T21:07:27Z"""
        )
        self.assertThat(result.output, Contains(expected_output))

    def test_gated_no_validations(self):
        self.client.login("dummy", "test correct password")

        result = self.run_command(["gated", "test-snap-with-no-validations"])

        self.assertThat(result.exit_code, Equals(0))
        expected_output = (
            "There are no validations for snap 'test-snap-with-no-validations'\n"
        )
        self.assertThat(result.output, Contains(expected_output))

    def test_no_login(self):
        raised = self.assertRaises(
            snapcraft.storeapi.errors.InvalidCredentialsError,
            self.run_command,
            ["gated", "notfound"],
        )

        self.assertThat(str(raised), Contains("Invalid credentials"))
