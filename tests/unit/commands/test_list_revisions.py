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

import logging
from textwrap import dedent

import fixtures
from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from . import FakeStoreCommandsBaseTestCase


class RevisionsCommandBaseTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.expected = [
            {
                "series": ["16"],
                "channels": [],
                "version": "2.0.1",
                "timestamp": "2016-09-27T19:23:40Z",
                "current_channels": ["beta", "edge"],
                "arch": "i386",
                "revision": 2,
            },
            {
                "series": ["16"],
                "channels": ["stable", "edge"],
                "version": "2.0.2",
                "timestamp": "2016-09-27T18:38:43Z",
                "current_channels": ["stable", "candidate", "beta"],
                "arch": "amd64",
                "revision": 1,
            },
        ]


class RevisionsCommandTestCase(RevisionsCommandBaseTestCase):

    scenarios = [
        ("list-revisions", dict(command_name="list-revisions")),
        ("revisions", dict(command_name="revisions")),
    ]

    def test_revisions_without_snap_raises_exception(self):
        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_revisions_without_login_must_ask(self):
        self.fake_store_account_info.mock.side_effect = [
            storeapi.errors.InvalidCredentialsError("error"),
            {"snaps": {"16": {"snap-test": {"snap-id": "snap-test-snap-id"}}}},
            {"snaps": {"16": {"snap-test": {"snap-id": "snap-test-snap-id"}}}},
            {"snaps": {"16": {"snap-test": {"snap-id": "snap-test-snap-id"}}}},
            {"snaps": {"16": {"snap-test": {"snap-id": "snap-test-snap-id"}}}},
        ]
        self.fake_store_revisions.mock.return_value = self.expected

        result = self.run_command(
            [self.command_name, "snap-test"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_revisions_with_3rd_party_snap(self):
        self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test"],
        )

    def test_revisions_with_3rd_party_snap_by_arch(self):
        self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test", "--arch=arm64"],
        )

    def test_revisions_by_unknown_arch(self):
        self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test", "--arch=some-arch"],
        )

    def test_revisions(self):
        self.fake_store_revisions.mock.return_value = self.expected

        result = self.run_command([self.command_name, "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Rev.    Uploaded              Arch    Version    Channels
            2       2016-09-27T19:23:40Z  i386    2.0.1      -
            1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge"""
                )
            ),
        )  # noqa
        self.fake_store_revisions.mock.assert_called_once_with(
            "snap-test-snap-id", "16", None
        )

    def test_revisions_by_arch(self):
        self.fake_store_revisions.mock.return_value = [
            rev for rev in self.expected if rev["arch"] == "amd64"
        ]

        result = self.run_command([self.command_name, "snap-test", "--arch=amd64"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Rev.    Uploaded              Arch    Version    Channels
            1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge"""
                )
            ),
        )
        self.fake_store_revisions.mock.assert_called_once_with(
            "snap-test-snap-id", "16", "amd64"
        )


class DeprecatedHistoryCommandTestCase(RevisionsCommandBaseTestCase):
    def test_history_with_deprecation_message(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        self.fake_store_revisions.mock.return_value = self.expected

        result = self.run_command(["history", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            fake_logger.output,
            Contains(
                "DEPRECATED: The 'history' command has been replaced by "
                "'list-revisions'."
            ),
        )
