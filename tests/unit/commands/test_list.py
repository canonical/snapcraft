# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2020 Canonical Ltd
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


class ListTest(FakeStoreCommandsBaseTestCase):

    command_name = "list"

    def test_command_without_login_must_ask(self):
        # TODO: look into why this many calls are done inside snapcraft.storeapi
        self.fake_store_account_info.mock.side_effect = [
            storeapi.http_clients.errors.InvalidCredentialsError("error"),
            {"account_id": "abcd", "snaps": dict()},
            {"account_id": "abcd", "snaps": dict()},
            {"account_id": "abcd", "snaps": dict()},
        ]

        result = self.run_command(
            [self.command_name], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_list_empty(self):
        self.fake_store_account_info.mock.return_value = {
            "account_id": "abcd",
            "snaps": dict(),
        }

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("There are no registered snaps."))

    def test_list_registered(self):
        self.command_name = "list-registered"
        self.fake_store_account_info.mock.return_value = {
            "account_id": "abcd",
            "snaps": dict(),
        }

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("There are no registered snaps."))

    def test_registered(self):
        self.command_name = "registered"
        self.fake_store_account_info.mock.return_value = {
            "account_id": "abcd",
            "snaps": dict(),
        }

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("There are no registered snaps."))

    def test_list_successfully(self):
        self.fake_store_account_info.mock.return_value = {
            "snaps": {
                "16": {
                    "foo": {
                        "status": "Approved",
                        "snap-id": "a_snap_id",
                        "private": False,
                        "since": "2016-12-12T01:01:01Z",
                        "price": "9.99",
                    },
                    "bar": {
                        "status": "ReviewPending",
                        "snap-id": "another_snap_id",
                        "private": True,
                        "since": "2016-12-12T01:01:01Z",
                        "price": None,
                    },
                    "baz": {
                        "status": "Approved",
                        "snap-id": "yet_another_snap_id",
                        "private": True,
                        "since": "2016-12-12T02:02:02Z",
                        "price": "6.66",
                    },
                    "boing": {
                        "status": "Approved",
                        "snap-id": "boing_snap_id",
                        "private": False,
                        "since": "2016-12-12T03:03:03Z",
                        "price": None,
                    },
                }
            }
        }

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Name    Since                 Visibility    Price    Notes
            baz     2016-12-12T02:02:02Z  private       6.66     -
            boing   2016-12-12T03:03:03Z  public        -        -
            foo     2016-12-12T01:01:01Z  public        9.99     -"""
                )
            ),
        )
