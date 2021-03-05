# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2021 Canonical Ltd
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

import fixtures
from testtools.matchers import Contains, Equals

import snapcraft.storeapi.errors
from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase


class CloseCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(
            fixtures.MockPatchObject(
                storeapi._dashboard_api.DashboardAPI,
                "close_channels",
                return_value=(list(), dict()),
            )
        )

    def test_close_missing_permission(self):
        self.fake_store_account_info.mock.return_value = {
            "account_id": "abcd",
            "snaps": {},
        }

        raised = self.assertRaises(
            snapcraft.storeapi.errors.StoreChannelClosingPermissionError,
            self.run_command,
            ["close", "foo", "beta"],
        )

        self.assertThat(
            str(raised),
            Equals(
                "Your account lacks permission to close channels for this snap. "
                "Make sure the logged in account has upload permissions on "
                "'foo' in series '16'."
            ),
        )

    def test_close(self):
        result = self.run_command(["close", "snap-test", "2.1/candidate"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.1      amd64   stable     -          -
                             candidate  -          -
                             beta       10         19
                             edge       ↑          ↑

            The 2.1/candidate channel is now closed."""
                )
            ),
        )

    def test_close_no_revisions(self):
        self.channel_map.channel_map = list()

        result = self.run_command(["close", "snap-test", "2.1/candidate"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output.strip(), Equals("The 2.1/candidate channel is now closed.")
        )

    def test_close_multiple_channels(self):
        result = self.run_command(["close", "snap-test", "2.1/stable", "2.1/edge/test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.1      amd64   stable     -          -
                             candidate  -          -
                             beta       10         19
                             edge       ↑          ↑

            The 2.1/stable and 2.1/edge/test channels are now closed."""
                )
            ),
        )
