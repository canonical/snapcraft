# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019 Canonical Ltd
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

from unittest import mock

from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from . import CommandBaseTestCase


class PromoteCommandTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch.object(storeapi.StoreClient, "get_snap_status")
        self.mock_status = patcher.start()
        self.addCleanup(patcher.stop)
        self.mock_status.return_value = {
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {
                                "channel": "candidate",
                                "info": "specific",
                                "revision": 1,
                                "version": "0.1",
                            },
                            {
                                "channel": "beta",
                                "info": "specific",
                                "revision": 2,
                                "version": "0.1",
                            },
                            {
                                "channel": "edge",
                                "info": "specific",
                                "revision": 5,
                                "version": "0.1",
                            },
                        ]
                    }
                }
            }
        }
        patcher = mock.patch.object(storeapi.StoreClient, "release")
        self.mock_release = patcher.start()
        self.addCleanup(patcher.stop)

    def test_upload_without_snap_must_raise_exception(self):
        result = self.run_command(["promote"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_promote_edge(self):
        result = self.run_command(
            [
                "promote",
                "test-snap",
                "--from-channel",
                "edge",
                "--to-channel",
                "candidate",
            ]
        )

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output,
            Contains("'edge' is not a valid set value for --from-channel."),
        )

    def test_promote_same(self):
        result = self.run_command(
            [
                "promote",
                "test-snap",
                "--from-channel",
                "candidate",
                "--to-channel",
                "candidate",
            ]
        )

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output,
            Contains("--from-channel and --to-channel cannot be the same."),
        )

    def test_promote_confirm_yes(self):
        self.mock_release.return_value = {
            "opened_channels": ["candidate"],
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {
                                "channel": "candidate",
                                "info": "specific",
                                "revision": 1,
                                "version": "0.1",
                            },
                            {
                                "channel": "beta",
                                "info": "specific",
                                "revision": 2,
                                "version": "0.1",
                            },
                            {
                                "channel": "edge",
                                "info": "specific",
                                "revision": 5,
                                "version": "0.1",
                            },
                        ]
                    }
                }
            },
        }

        result = self.run_command(
            [
                "promote",
                "test-snap",
                "--from-channel",
                "beta",
                "--to-channel",
                "candidate",
            ],
            input="y\n",
        )

        self.assertThat(result.exit_code, Equals(0))
        self.mock_release.assert_called_once_with("test-snap", "2", ["candidate"])

    def test_promote_yes_option(self):
        self.mock_release.return_value = {
            "opened_channels": ["candidate"],
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "amd64": [
                            {"channel": "stable", "info": "none"},
                            {
                                "channel": "candidate",
                                "info": "specific",
                                "revision": 1,
                                "version": "0.1",
                            },
                            {
                                "channel": "beta",
                                "info": "specific",
                                "revision": 2,
                                "version": "0.1",
                            },
                            {
                                "channel": "edge",
                                "info": "specific",
                                "revision": 5,
                                "version": "0.1",
                            },
                        ]
                    }
                }
            },
        }

        result = self.run_command(
            [
                "promote",
                "test-snap",
                "--from-channel",
                "beta",
                "--to-channel",
                "candidate",
                "--yes",
            ]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.mock_release.assert_called_once_with("test-snap", "2", ["candidate"])

    def test_promote_confirm_no(self):
        result = self.run_command(
            [
                "promote",
                "test-snap",
                "--from-channel",
                "beta",
                "--to-channel",
                "candidate",
            ],
            input="n\n",
        )

        self.assertThat(result.exit_code, Equals(0))
        self.mock_release.assert_not_called()
