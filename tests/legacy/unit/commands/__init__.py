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
import subprocess
from pathlib import PosixPath
from textwrap import dedent
from unittest import mock

import craft_store
import fixtures
import pytest
import requests
from click.testing import CliRunner

from snapcraft_legacy import storeapi
from snapcraft_legacy.cli._runner import run
from snapcraft_legacy.storeapi import metrics
from snapcraft_legacy.storeapi.v2.releases import Releases
from tests.legacy import fixture_setup, unit

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
    if (
        command[0].endswith("unsquashfs")
        or command[0].endswith("xdelta3")
        or command[0].endswith("file")
    ):
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
    elif command[0].endswith("snap") and command[1] == "sign-build":
        return b"Mocked assertion"
    else:
        raise AssertionError("Unhandled command: {}".format(command))


@pytest.mark.usefixtures("memory_keyring")
class CommandBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.runner = CliRunner()

    def run_command(self, args, **kwargs):
        # For click testing, runner will overwrite the descriptors for stdio -
        # ensure TTY always appears connected.
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft_legacy.cli.echo.is_tty_connected", return_value=True
            )
        )

        with mock.patch("sys.argv", args):
            return self.runner.invoke(run, args, catch_exceptions=False, **kwargs)


class LifecycleCommandsBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT"))

        self.fake_lifecycle_clean = fixtures.MockPatch(
            "snapcraft_legacy.internal.lifecycle.clean"
        )
        self.useFixture(self.fake_lifecycle_clean)

        self.fake_lifecycle_execute = fixtures.MockPatch(
            "snapcraft_legacy.internal.lifecycle.execute"
        )
        self.useFixture(self.fake_lifecycle_execute)

        self.fake_pack = fixtures.MockPatch("snapcraft_legacy.cli.lifecycle._pack")
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
            "snapcraft_legacy.internal.build_providers.get_provider_for",
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

        self.client.login(email="dummy", password="test correct password", ttl=1)


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

        self.fake_store_logout = fixtures.MockPatchObject(
            storeapi.StoreClient, "logout"
        )
        self.useFixture(self.fake_store_logout)

        self.fake_store_register = fixtures.MockPatchObject(
            storeapi._dashboard_api.DashboardAPI, "register"
        )
        self.useFixture(self.fake_store_register)

        self.fake_store_account_info_data = {
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
        }

        self.fake_store_account_info = fixtures.MockPatchObject(
            storeapi._dashboard_api.DashboardAPI,
            "get_account_information",
            return_value=self.fake_store_account_info_data,
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

        self.metrics = metrics.MetricsResults(
            metrics=[
                metrics.MetricResults(
                    status=metrics.MetricsStatus["OK"],
                    snap_id="test-snap-id",
                    metric_name="daily_device_change",
                    buckets=["2021-01-01", "2021-01-02", "2021-01-03"],
                    series=[
                        metrics.Series(
                            name="continued",
                            values=[10, 11, 12],
                            currently_released=None,
                        ),
                        metrics.Series(
                            name="lost", values=[1, 2, 3], currently_released=None
                        ),
                        metrics.Series(
                            name="new", values=[2, 3, 4], currently_released=None
                        ),
                    ],
                )
            ]
        )
        self.fake_store_get_metrics = fixtures.MockPatchObject(
            storeapi.StoreClient, "get_metrics", return_value=self.metrics
        )
        self.useFixture(self.fake_store_get_metrics)

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
        # Mock the snap command, pass through a select few.
        self.fake_check_output = fixtures.MockPatch(
            "subprocess.check_output", side_effect=mock_check_output
        )
        self.useFixture(self.fake_check_output)

        # Pretend that the snap command is available
        self.fake_package_installed = fixtures.MockPatch(
            "snapcraft_legacy.internal.repo.Repo.is_package_installed",
            return_value=True,
        )
        self.useFixture(self.fake_package_installed)


class FakeResponse(requests.Response):
    def __init__(self, content, status_code):
        self._content = content
        self.status_code = status_code

    @property
    def content(self):
        return self._content

    @property
    def ok(self):
        return self.status_code == 200

    def json(self):
        return json.loads(self._content)  # type: ignore

    @property
    def reason(self):
        return self._content

    @property
    def text(self):
        return self.content


FAKE_UNAUTHORIZED_ERROR = craft_store.errors.StoreServerError(
    FakeResponse(
        status_code=requests.codes.unauthorized, content=json.dumps({"error": "error"})
    )
)
