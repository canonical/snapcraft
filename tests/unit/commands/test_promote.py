# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019-2020 Canonical Ltd
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

from testtools.matchers import Contains, Equals

from . import FakeStoreCommandsBaseTestCase


class PromoteCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_store_status.mock.return_value = {
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
        self.fake_store_release.mock.return_value = {"opened_channels": ["candidate"]}

    def test_upload_without_snap_must_raise_exception(self):
        result = self.run_command(["promote"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_promote_edge_with_yes(self):
        result = self.run_command(
            [
                "promote",
                "test-snap",
                "--from-channel",
                "edge",
                "--to-channel",
                "candidate",
                "--yes",
            ]
        )

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output,
            Contains(
                "'edge' is not a valid set value for --from-channel when using --yes."
            ),
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
        result = self.run_command(
            [
                "promote",
                "snap-test",
                "--from-channel",
                "beta",
                "--to-channel",
                "candidate",
            ],
            input="y\n",
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_store_release.mock.assert_called_once_with(
            snap_name="snap-test", revision="2", channels=["candidate"]
        )

    def test_promote_yes_option(self):
        result = self.run_command(
            [
                "promote",
                "snap-test",
                "--from-channel",
                "beta",
                "--to-channel",
                "candidate",
                "--yes",
            ]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_store_release.mock.assert_called_once_with(
            snap_name="snap-test", revision="2", channels=["candidate"]
        )

    def test_promote_confirm_no(self):
        result = self.run_command(
            [
                "promote",
                "snap-test",
                "--from-channel",
                "beta",
                "--to-channel",
                "candidate",
            ],
            input="n\n",
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_store_release.mock.assert_not_called()
