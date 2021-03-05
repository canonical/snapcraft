# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2021 Canonical Ltd
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

import json
from pathlib import PosixPath
import subprocess
from textwrap import dedent
from unittest import mock

import fixtures
from click.testing import CliRunner

from snapcraft import storeapi
from snapcraft.cli._runner import run
from snapcraft.storeapi.v2.channel_map import ChannelMap
from snapcraft.storeapi.v2.releases import Releases
from tests import fixture_setup, unit

_sample_keys = [
    {
        "name": "default",
        "sha3-384": "vdEeQvRxmZ26npJCFaGnl-VfGz0lU2jZZkWp_s7E-RxVCNtH2_mtjcxq2NkDKkIp",
    },
    {
        "name": "another",
        "sha3-384": "JsfToV5hO2eN9l89pYYCKXUioTERrZIIHUgQQd47jW8YNNBskupiIjWYd3KXLY_D",
    },
]


def get_sample_key(name):
    for key in _sample_keys:
        if key["name"] == name:
            return key
    raise KeyError(name)


original_check_output = subprocess.check_output


def mock_check_output(command, *args, **kwargs):
    if isinstance(command[0], PosixPath):
        command[0] = str(command[0])
    if command[0].endswith("unsquashfs") or command[0].endswith("xdelta3"):
        return original_check_output(command, *args, **kwargs)
    elif command[0].endswith("snap") and command[1:] == ["keys", "--json"]:
        return json.dumps(_sample_keys)
    elif command[0].endswith("snap") and command[1] == "export-key":
        if not command[2].startswith("--account="):
            raise AssertionError("Unhandled command: {}".format(command))
        account_id = command[2][len("--account=") :]
        name = command[3]
        # This isn't a full account-key-request assertion, but it's enough
        # for testing.
        return dedent(
            """\
            type: account-key-request
            account-id: {account_id}
            name: {name}
            public-key-sha3-384: {sha3_384}
            """
        ).format(
            account_id=account_id, name=name, sha3_384=get_sample_key(name)["sha3-384"]
        )
    elif command[0].endswith("snap") and command[1:] == [
        "create-key",
        "new-key",
    ]:
        pass
    else:
        raise AssertionError("Unhandled command: {}".format(command))


class CommandBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.runner = CliRunner()

    def run_command(self, args, **kwargs):
        # For click testing, runner will overwrite the descriptors for stdio -
        # ensure TTY always appears connected.
        self.useFixture(
            fixtures.MockPatch("snapcraft.cli.echo.is_tty_connected", return_value=True)
        )

        with mock.patch("sys.argv", args):
            return self.runner.invoke(run, args, catch_exceptions=False, **kwargs)


class LifecycleCommandsBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT"))

        self.fake_lifecycle_clean = fixtures.MockPatch(
            "snapcraft.internal.lifecycle.clean"
        )
        self.useFixture(self.fake_lifecycle_clean)

        self.fake_lifecycle_execute = fixtures.MockPatch(
            "snapcraft.internal.lifecycle.execute"
        )
        self.useFixture(self.fake_lifecycle_execute)

        self.fake_pack = fixtures.MockPatch("snapcraft.cli.lifecycle._pack")
        self.useFixture(self.fake_pack)

        self.snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path,
            parts={
                "part0": {"plugin": "nil"},
                "part1": {"plugin": "nil"},
                "part2": {"plugin": "nil"},
            },
        )
        self.useFixture(self.snapcraft_yaml)

        self.provider_class_mock = mock.MagicMock()
        self.provider_mock = mock.MagicMock()
        self.provider_class_mock.return_value.__enter__.return_value = (
            self.provider_mock
        )

        self.fake_get_provider_for = fixtures.MockPatch(
            "snapcraft.internal.build_providers.get_provider_for",
            return_value=self.provider_class_mock,
        )
        self.useFixture(self.fake_get_provider_for)

    def assert_clean_not_called(self):
        self.fake_lifecycle_clean.mock.assert_not_called()
        self.provider_mock.clean.assert_not_called()
        self.provider_mock.clean_project.assert_not_called()


class StoreCommandsBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        self.fake_store = fixture_setup.FakeStore()
        self.useFixture(self.fake_store)
        self.client = storeapi.StoreClient()


class FakeStoreCommandsBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        # Our experimental environment variable is sticky
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_EXPERIMENTAL_PROGRESSIVE_RELEASES", None
            )
        )

        self.fake_store_login = fixtures.MockPatchObject(storeapi.StoreClient, "login")
        self.useFixture(self.fake_store_login)

        self.fake_store_register = fixtures.MockPatchObject(
            storeapi._dashboard_api.DashboardAPI, "register"
        )
        self.useFixture(self.fake_store_register)

        self.fake_store_account_info = fixtures.MockPatchObject(
            storeapi._dashboard_api.DashboardAPI,
            "get_account_information",
            return_value={
                "account_id": "abcd",
                "account_keys": list(),
                "snaps": {
                    "16": {
                        "snap-test": {
                            "snap-id": "snap-test-snap-id",
                            "status": "Approved",
                            "private": False,
                            "since": "2016-12-12T01:01Z",
                            "price": "0",
                        },
                        "basic": {
                            "snap-id": "basic-snap-id",
                            "status": "Approved",
                            "private": False,
                            "since": "2016-12-12T01:01Z",
                            "price": "0",
                        },
                    }
                },
            },
        )
        self.useFixture(self.fake_store_account_info)

        self.fake_store_status = fixtures.MockPatchObject(
            storeapi._dashboard_api.DashboardAPI, "snap_status", return_value=dict()
        )
        self.useFixture(self.fake_store_status)

        self.fake_store_release = fixtures.MockPatchObject(
            storeapi.StoreClient, "release"
        )
        self.useFixture(self.fake_store_release)

        self.fake_store_register_key = fixtures.MockPatchObject(
            storeapi._dashboard_api.DashboardAPI, "register_key"
        )
        self.useFixture(self.fake_store_register_key)

        # channel-map endpoint
        self.channel_map = ChannelMap.unmarshal(
            {
                "channel-map": [
                    {
                        "architecture": "amd64",
                        "channel": "2.1/beta",
                        "expiration-date": None,
                        "revision": 19,
                        "progressive": {
                            "paused": None,
                            "percentage": None,
                            "current-percentage": None,
                        },
                    },
                    {
                        "architecture": "amd64",
                        "channel": "2.0/beta",
                        "expiration-date": None,
                        "revision": 18,
                        "progressive": {
                            "paused": None,
                            "percentage": None,
                            "current-percentage": None,
                        },
                    },
                ],
                "revisions": [
                    {"architectures": ["amd64"], "revision": 19, "version": "10"},
                    {"architectures": ["amd64"], "revision": 18, "version": "10"},
                ],
                "snap": {
                    "name": "snap-test",
                    "channels": [
                        {
                            "branch": None,
                            "fallback": None,
                            "name": "2.1/stable",
                            "risk": "stable",
                            "track": "2.1",
                        },
                        {
                            "branch": None,
                            "fallback": "2.1/stable",
                            "name": "2.1/candidate",
                            "risk": "candidate",
                            "track": "2.1",
                        },
                        {
                            "branch": None,
                            "fallback": "2.1/candidate",
                            "name": "2.1/beta",
                            "risk": "beta",
                            "track": "2.1",
                        },
                        {
                            "branch": None,
                            "fallback": "2.1/beta",
                            "name": "2.1/edge",
                            "risk": "edge",
                            "track": "2.1",
                        },
                        {
                            "branch": None,
                            "fallback": None,
                            "name": "2.0/stable",
                            "risk": "stable",
                            "track": "2.0",
                        },
                        {
                            "branch": None,
                            "fallback": "2.0/stable",
                            "name": "2.0/candidate",
                            "risk": "candidate",
                            "track": "2.0",
                        },
                        {
                            "branch": None,
                            "fallback": "2.0/candidate",
                            "name": "2.0/beta",
                            "risk": "beta",
                            "track": "2.0",
                        },
                        {
                            "branch": None,
                            "fallback": "2.0/beta",
                            "name": "2.0/edge",
                            "risk": "edge",
                            "track": "2.0",
                        },
                    ],
                    "default-track": "2.1",
                    "tracks": [
                        {
                            "name": "2.0",
                            "status": "default",
                            "creation-date": "2019-10-17T14:11:59Z",
                            "version-pattern": "2\\.*",
                        },
                        {
                            "name": "latest",
                            "status": "active",
                            "creation-date": None,
                            "version-pattern": None,
                        },
                    ],
                },
            }
        )
        self.fake_store_get_snap_channel_map = fixtures.MockPatchObject(
            storeapi.StoreClient, "get_snap_channel_map", return_value=self.channel_map
        )
        self.useFixture(self.fake_store_get_snap_channel_map)

        self.releases = Releases.unmarshal(
            {
                "revisions": [
                    {
                        "architectures": ["i386"],
                        "base": "core20",
                        "build_url": None,
                        "confinement": "strict",
                        "created_at": " 2016-09-27T19:23:40Z",
                        "grade": "stable",
                        "revision": 2,
                        "sha3-384": "a9060ef4872ccacbfa440617a76fcd84967896b28d0d1eb7571f00a1098d766e7e93353b084ba6ad841d7b14b95ede48",
                        "size": 20,
                        "status": "Published",
                        "version": "2.0.1",
                    },
                    {
                        "architectures": ["amd64"],
                        "base": "core20",
                        "build_url": None,
                        "confinement": "strict",
                        "created_at": "2016-09-27T18:38:43Z",
                        "grade": "stable",
                        "revision": 1,
                        "sha3-384": "a9060ef4872ccacbfa440617a76fcd84967896b28d0d1eb7571f00a1098d766e7e93353b084ba6ad841d7b14b95ede48",
                        "size": 20,
                        "status": "Published",
                        "version": "2.0.2",
                    },
                ],
                "releases": [
                    {
                        "architecture": "amd64",
                        "branch": None,
                        "channel": "latest/stable",
                        "expiration-date": None,
                        "revision": 1,
                        "risk": "stable",
                        "track": "latest",
                        "when": "2020-02-12T17:51:40.891996Z",
                    },
                    {
                        "architecture": "i386",
                        "branch": None,
                        "channel": "latest/stable",
                        "expiration-date": None,
                        "revision": None,
                        "risk": "stable",
                        "track": "latest",
                        "when": "2020-02-11T17:51:40.891996Z",
                    },
                    {
                        "architecture": "amd64",
                        "branch": None,
                        "channel": "latest/edge",
                        "expiration-date": None,
                        "revision": 1,
                        "risk": "stable",
                        "track": "latest",
                        "when": "2020-01-12T17:51:40.891996Z",
                    },
                ],
            }
        )
        self.fake_store_get_releases = fixtures.MockPatchObject(
            storeapi.StoreClient, "get_snap_releases", return_value=self.releases
        )
        self.useFixture(self.fake_store_get_releases)

        # Uploading
        self.mock_tracker = mock.Mock(storeapi._status_tracker.StatusTracker)
        self.mock_tracker.track.return_value = {
            "code": "ready_to_release",
            "processed": True,
            "can_release": True,
            "url": "/fake/url",
            "revision": 19,
        }
        self.fake_store_upload_precheck = fixtures.MockPatchObject(
            storeapi.StoreClient, "upload_precheck"
        )
        self.useFixture(self.fake_store_upload_precheck)

        self.fake_store_upload = fixtures.MockPatchObject(
            storeapi.StoreClient, "upload", return_value=self.mock_tracker
        )
        self.useFixture(self.fake_store_upload)

        # Mock the snap command, pass through a select few.
        self.fake_check_output = fixtures.MockPatch(
            "subprocess.check_output", side_effect=mock_check_output
        )
        self.useFixture(self.fake_check_output)

        # Pretend that the snap command is available
        self.fake_package_installed = fixtures.MockPatch(
            "snapcraft.internal.repo.Repo.is_package_installed", return_value=True
        )
        self.useFixture(self.fake_package_installed)
