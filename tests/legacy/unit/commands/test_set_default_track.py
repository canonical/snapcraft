# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2020 Canonical Ltd
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

from snapcraft_legacy import storeapi

from . import FAKE_UNAUTHORIZED_ERROR, FakeStoreCommandsBaseTestCase


class SetDefaultTrackCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_metadata = fixtures.MockPatchObject(
            storeapi.StoreClient, "upload_metadata"
        )
        self.useFixture(self.fake_metadata)

    def test_set_default_track_without_snap_raises_exception(self):
        result = self.run_command(["set-default-track"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_set_default_track_without_login_must_ask(self):
        self.fake_metadata.mock.side_effect = [
            FAKE_UNAUTHORIZED_ERROR,
            None,
        ]

        result = self.run_command(
            ["set-default-track", "snap-test", "2.0"],
            input="user@example.com\nsecret\n",
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_set_default_track(self):
        result = self.run_command(["set-default-track", "snap-test", "2.0"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_metadata.mock.assert_called_once_with(
            snap_name="snap-test", metadata=dict(default_track="2.0"), force=True
        )
