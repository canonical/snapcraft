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
from textwrap import dedent
from unittest import mock

from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from . import StoreCommandsBaseTestCase


class ListRegisteredTestCase(StoreCommandsBaseTestCase):

    scenarios = [
        ("list-registered", {"command_name": "list-registered"}),
        ("registered alias", {"command_name": "registered"}),
    ]

    def test_list_registered_without_login(self):
        raised = self.assertRaises(
            storeapi.errors.InvalidCredentialsError,
            self.run_command,
            [self.command_name],
        )

        self.assertThat(str(raised), Contains("Invalid credentials"))

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    def test_list_registered_empty(self, mock_get_account_information):
        mock_get_account_information.return_value = {"snaps": {}}

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Contains("There are no registered snaps for series '16'.")
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    def test_list_registered_successfully(self, mock_get_account_information):
        mock_get_account_information.return_value = {
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
