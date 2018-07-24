# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2017 Canonical Ltd
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

import snapcraft.storeapi.errors
from snapcraft import storeapi
from . import CommandBaseTestCase


class CloseCommandTestCase(CommandBaseTestCase):
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    def test_close_missing_permission(self, mock_get_account_info):
        mock_get_account_info.return_value = {"account_id": "abcd", "snaps": {}}

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

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch.object(storeapi._sca_client.SCAClient, "close_channels")
    def test_close_basic(self, mock_close_channels, mock_get_account_info):
        mock_get_account_info.return_value = {
            "snaps": {"16": {"basic": {"snap-id": "snap-id"}}}
        }
        closed_channels = ["beta"]
        channel_map_tree = {
            "latest": {
                "16": {
                    "amd64": [
                        {"channel": "stable", "info": "none"},
                        {"channel": "candidate", "info": "none"},
                        {
                            "channel": "beta",
                            "info": "specific",
                            "version": "1.1",
                            "revision": 42,
                        },
                        {"channel": "edge", "info": "tracking"},
                    ]
                }
            }
        }
        mock_close_channels.side_effect = [(closed_channels, channel_map_tree)]

        result = self.run_command(["close", "basic", "beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     -          -
                             candidate  -          -
                             beta       1.1        42
                             edge       ^          ^

            \x1b[0;32mThe beta channel is now closed.\x1b[0m"""
                )
            ),
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch.object(storeapi._sca_client.SCAClient, "close_channels")
    def test_close_multiple_channels(self, mock_close_channels, mock_get_account_info):
        mock_get_account_info.return_value = {
            "snaps": {"16": {"basic": {"snap-id": "snap-id"}}}
        }
        closed_channels = ["beta", "edge"]
        channel_map_tree = {
            "latest": {
                "16": {
                    "amd64": [
                        {"channel": "stable", "info": "none"},
                        {
                            "channel": "candidate",
                            "info": "specific",
                            "version": "1.1",
                            "revision": 42,
                        },
                        {"channel": "beta", "info": "tracking"},
                        {"channel": "edge", "info": "tracking"},
                    ]
                }
            }
        }
        mock_close_channels.side_effect = [(closed_channels, channel_map_tree)]

        result = self.run_command(["close", "basic", "beta", "edge"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     -          -
                             candidate  1.1        42
                             beta       ^          ^
                             edge       ^          ^

            \x1b[0;32mThe beta and edge channels are now closed.\x1b[0m"""
                )
            ),
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch.object(storeapi._sca_client.SCAClient, "close_channels")
    def test_close_multiple_architectures(
        self, mock_close_channels, mock_get_account_info
    ):
        mock_get_account_info.return_value = {
            "snaps": {"16": {"basic": {"snap-id": "snap-id"}}}
        }
        closed_channels = ["beta"]
        channel_map_tree = {
            "latest": {
                "16": {
                    "amd64": [
                        {"channel": "stable", "info": "none"},
                        {"channel": "candidate", "info": "none"},
                        {
                            "channel": "beta",
                            "info": "specific",
                            "version": "1.1",
                            "revision": 42,
                        },
                        {"channel": "edge", "info": "tracking"},
                    ],
                    "armhf": [
                        {"channel": "stable", "info": "none"},
                        {
                            "channel": "beta",
                            "info": "specific",
                            "version": "1.2",
                            "revision": 24,
                        },
                        {"channel": "beta", "info": "tracking"},
                        {"channel": "edge", "info": "tracking"},
                    ],
                }
            }
        }
        mock_close_channels.side_effect = [(closed_channels, channel_map_tree)]

        result = self.run_command(["close", "basic", "beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     -          -
                             candidate  -          -
                             beta       1.1        42
                             edge       ^          ^
                     armhf   stable     -          -
                             beta       1.2        24
                             beta       ^          ^
                             edge       ^          ^

            \x1b[0;32mThe beta channel is now closed.\x1b[0m"""
                )
            ),
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch.object(storeapi._sca_client.SCAClient, "close_channels")
    def test_close_branches(self, mock_close_channels, mock_get_account_info):
        mock_get_account_info.return_value = {
            "snaps": {"16": {"basic": {"snap-id": "snap-id"}}}
        }
        closed_channels = ["stable/hotfix-1"]
        channel_map_tree = {
            "latest": {
                "16": {
                    "amd64": [
                        {"channel": "stable", "info": "none"},
                        {"channel": "candidate", "info": "none"},
                        {
                            "channel": "beta",
                            "info": "specific",
                            "version": "1.1",
                            "revision": 42,
                        },
                        {"channel": "edge", "info": "tracking"},
                        {
                            "channel": "stable/hotfix-2",
                            "info": "branch",
                            "version": "1.3",
                            "revision": 49,
                            "expires_at": "2017-05-21T18:52:14.578435",
                        },
                    ]
                }
            }
        }
        mock_close_channels.side_effect = [(closed_channels, channel_map_tree)]

        result = self.run_command(["close", "basic", "stable/hotfix-1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel          Version    Revision    Expires at
            latest   amd64   stable           -          -
                             candidate        -          -
                             beta             1.1        42
                             edge             ^          ^
                             stable/hotfix-2  1.3        49          2017-05-21T18:52:14.578435

            \x1b[0;32mThe stable/hotfix-1 channel is now closed.\x1b[0m"""
                )
            ),
        )  # noqa
