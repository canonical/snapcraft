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

from textwrap import dedent

from testtools.matchers import Contains, Equals

from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase


class ListTracksCommandTestCase(FakeStoreCommandsBaseTestCase):
    def test_list_tracks_without_snap_raises_exception(self):
        result = self.run_command(["list-tracks"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_list_tracks_without_login_must_ask(self):
        self.fake_store_get_snap_channel_map.mock.side_effect = [
            storeapi.http_clients.errors.InvalidCredentialsError("error"),
            self.channel_map,
        ]

        result = self.run_command(
            ["list-tracks", "snap-test"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_list_tracks(self):
        result = self.run_command(["list-tracks", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Name    Status    Creation-Date         Version-Pattern
            latest  active    -                     -
            2.0     default   2019-10-17T14:11:59Z  2\\.*
            """
                )
            ),
        )
