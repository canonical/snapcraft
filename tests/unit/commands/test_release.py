# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2019 Canonical Ltd
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


class ReleaseCommandTestCase(FakeStoreCommandsBaseTestCase):
    def test_upload_without_snap_must_raise_exception(self):
        result = self.run_command(["release"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_release_snap(self):
        self.fake_store_release.mock.return_value = {
            "opened_channels": ["beta"],
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "revision": 19,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {"channel": "edge", "info": "tracking"},
                        ]
                    }
                }
            },
        }

        result = self.run_command(["release", "nil-snap", "19", "beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     -          -
                             candidate  -          -
                             beta       0          19
                             edge       ^          ^
            \x1b[0;32mThe 'beta' channel is now open.\x1b[0m"""
                )
            ),
        )
        self.fake_store_release.mock.assert_called_once_with("nil-snap", "19", ["beta"])

    def test_release_snap_with_lts_channel(self):
        self.fake_store_release.mock.return_value = {
            "opened_channels": ["2.1/beta"],
            "channel_map_tree": {
                "2.1": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "revision": 19,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {"channel": "edge", "info": "tracking"},
                        ]
                    }
                }
            },
        }

        result = self.run_command(["release", "nil-snap", "19", "2.1/beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.1      amd64   stable     -          -
                             candidate  -          -
                             beta       0          19
                             edge       ^          ^
            \x1b[0;32mThe '2.1/beta' channel is now open.\x1b[0m"""
                )
            ),
        )
        self.fake_store_release.mock.assert_called_once_with(
            "nil-snap", "19", ["2.1/beta"]
        )

    def test_release_snap_with_branch(self):
        self.fake_store_release.mock.return_value = {
            "opened_channels": ["stable/hotfix1"],
            "channel_map_tree": {
                "2.1": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "revision": 19,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {"channel": "edge", "info": "tracking"},
                            {
                                "channel": "stable/hotfix1",
                                "info": "branch",
                                "revision": 20,
                                "version": "1",
                                "expires_at": "2017-05-21T18:52:14.578435",
                            },
                        ]
                    }
                }
            },
        }

        result = self.run_command(["release", "nil-snap", "20", "stable/hotfix1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel         Version    Revision    Expires at
            2.1      amd64   stable          -          -
                             candidate       -          -
                             beta            0          19
                             edge            ^          ^
                             stable/hotfix1  1          20          2017-05-21T18:52:14.578435
            \x1b[0;32mThe 'stable/hotfix1' channel is now open.\x1b[0m"""
                )
            ),
        )  # noqa
        self.fake_store_release.mock.assert_called_once_with(
            "nil-snap", "20", ["stable/hotfix1"]
        )

    def test_release_snap_opens_more_than_one_channel(self):
        self.fake_store_release.mock.return_value = {
            "opened_channels": ["stable", "beta", "edge"],
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "revision": 19,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {"channel": "edge", "info": "tracking"},
                        ]
                    }
                }
            },
        }

        result = self.run_command(["release", "nil-snap", "19", "beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     -          -
                             candidate  -          -
                             beta       0          19
                             edge       ^          ^
            \x1b[0;32mThe 'stable', 'beta' and 'edge' channels are now open.\x1b[0m"""
                )
            ),
        )  # noqa
        self.fake_store_release.mock.assert_called_once_with("nil-snap", "19", ["beta"])

    def test_release_with_bad_channel_info(self):
        self.fake_store_release.mock.return_value = {
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "fake-bad-channel-info"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "revision": 19,
                                "channel": "beta",
                                "version": "0",
                                "info": "specific",
                            },
                            {"channel": "edge", "info": "tracking"},
                        ]
                    }
                }
            }
        }

        result = self.run_command(["release", "nil-snap", "19", "beta"])

        self.assertThat(result.exit_code, Equals(0))

        self.fake_store_release.mock.assert_called_once_with("nil-snap", "19", ["beta"])

        # output will include the channel with no info, but there will be a log
        # in error alerting the problem
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable
                             candidate  -          -
                             beta       0          19
                             edge       ^          ^"""
                )
            ),
        )
        self.assertThat(
            result.output,
            Contains(
                "Unexpected channel info: 'fake-bad-channel-info' in channel stable"
            ),
        )

    def test_release_without_login_must_ask(self):
        self.fake_store_release.mock.side_effect = [
            storeapi.errors.InvalidCredentialsError("error"),
            {
                "opened_channels": ["beta"],
                "channel_map_tree": {
                    "latest": {
                        "16": {
                            "amd64": [
                                {"channel": "stable", "info": "none"},
                                {"channel": "candidate", "info": "none"},
                                {
                                    "revision": 19,
                                    "channel": "beta",
                                    "version": "0",
                                    "info": "specific",
                                },
                                {"channel": "edge", "info": "tracking"},
                            ]
                        }
                    }
                },
            },
        ]

        result = self.run_command(
            ["release", "nil-snap", "19", "beta"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )
