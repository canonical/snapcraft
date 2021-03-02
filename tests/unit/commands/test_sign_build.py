# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2021 Canonical Ltd
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
import os
import shutil
import subprocess
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, FileExists, Not

import tests
from snapcraft import internal, storeapi

from . import CommandBaseTestCase


class SnapTest(fixtures.TempDir):
    """Test fixture for copying testing snaps in a temporary directory.

    So they can be manipulated in isolation and all the testing mess
    gets cleaned up automatically.
    """

    data_dir = os.path.join(os.path.dirname(tests.__file__), "data")

    def __init__(self, test_snap_name):
        super(SnapTest, self).__init__()
        self.test_snap_name = test_snap_name

    def _setUp(self):
        super(SnapTest, self)._setUp()
        test_snap_path = os.path.join(self.data_dir, self.test_snap_name)
        self.snap_path = os.path.join(self.path, self.test_snap_name)
        shutil.copyfile(test_snap_path, self.snap_path)


class SignBuildTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        self.snap_test = SnapTest("test-snap.snap")
        self.useFixture(self.snap_test)

    def test_sign_build_nonexisting_snap(self):
        result = self.run_command(["sign-build", "nonexisting.snap"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output,
            Contains(
                "Error: Invalid value for '<snap-file>': File 'nonexisting.snap' does not exist.\n"
            ),
        )

    def test_sign_build_invalid_snap(self):
        snap_path = os.path.join(
            os.path.dirname(tests.__file__), "data", "invalid.snap"
        )

        raised = self.assertRaises(
            internal.errors.SnapDataExtractionError,
            self.run_command,
            ["sign-build", snap_path],
        )

        self.assertThat(str(raised), Contains("Cannot read data from snap"))

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_missing_account_info(
        self, mock_get_snap_data, mock_get_account_info,
    ):
        mock_get_account_info.return_value = {"account_id": "abcd", "snaps": {}}
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}

        raised = self.assertRaises(
            storeapi.errors.StoreBuildAssertionPermissionError,
            self.run_command,
            ["sign-build", self.snap_test.snap_path],
        )

        self.assertThat(
            str(raised),
            Equals(
                "Your account lacks permission to assert builds for this "
                "snap. Make sure you are logged in as the publisher of "
                "'test-snap' for series '16'."
            ),
        )

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_no_usable_keys(
        self, mock_get_snap_data, mock_get_account_info,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}

        self.useFixture(
            fixtures.MockPatch("subprocess.check_output", return_value="[]".encode())
        )

        raised = self.assertRaises(
            storeapi.errors.NoKeysError,
            self.run_command,
            ["sign-build", self.snap_test.snap_path],
        )

        self.assertThat(str(raised), Contains("You have no usable keys."))
        self.assertThat(
            str(raised),
            Contains(
                "Please create at least one key with `snapcraft create-key` "
                "for use with snap."
            ),
        )
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(snap_build_path, Not(FileExists()))

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_no_usable_named_key(
        self, mock_get_snap_data, mock_get_account_info,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        self.useFixture(
            fixtures.MockPatch(
                "subprocess.check_output", return_value='[{"name": "default"}]'.encode()
            )
        )

        raised = self.assertRaises(
            storeapi.errors.NoSuchKeyError,
            self.run_command,
            ["sign-build", "--key-name", "zoing", self.snap_test.snap_path],
        )

        self.assertThat(str(raised), Contains("You have no usable key named 'zoing'."))
        self.assertThat(
            str(raised),
            Contains("See the keys available in your system with `snapcraft keys`."),
        )

        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(snap_build_path, Not(FileExists()))

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_unregistered_key(
        self, mock_get_snap_data, mock_get_account_info,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "another_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        self.useFixture(
            fixtures.MockPatch(
                "subprocess.check_output",
                return_value='[{"name": "default", "sha3-384": "a_hash"}]'.encode(),
            )
        )

        raised = self.assertRaises(
            storeapi.errors.KeyNotRegisteredError,
            self.run_command,
            ["sign-build", self.snap_test.snap_path],
        )

        self.assertThat(
            str(raised), Contains("The key 'default' is not registered in the Store.")
        )
        self.assertThat(
            str(raised),
            Contains(
                "Please register it with `snapcraft register-key 'default'` "
                "before signing and uploading signatures to the Store."
            ),
        )
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(snap_build_path, Not(FileExists()))

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_snapd_failure(
        self, mock_get_snap_data, mock_get_account_info,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "a_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        self.useFixture(
            fixtures.MockPatch(
                "subprocess.check_output",
                side_effect=[
                    '[{"name": "default", "sha3-384": "a_hash"}]'.encode(),
                    subprocess.CalledProcessError(1, ["a", "b"]),
                ],
            )
        )

        raised = self.assertRaises(
            storeapi.errors.SignBuildAssertionError,
            self.run_command,
            ["sign-build", self.snap_test.snap_path],
        )

        self.assertThat(
            str(raised),
            Contains(
                "Failed to sign build assertion for {!r}".format(
                    self.snap_test.snap_path
                )
            ),
        )
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(snap_build_path, Not(FileExists()))

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_locally_successfully(
        self, mock_get_snap_data, mock_get_account_info,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        fake_check_output = fixtures.MockPatch(
            "subprocess.check_output",
            side_effect=['[{"name": "default"}]'.encode(), b"Mocked assertion"],
        )
        self.useFixture(fake_check_output)

        result = self.run_command(["sign-build", self.snap_test.snap_path, "--local"])

        self.assertThat(result.exit_code, Equals(0))
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(
            result.output,
            Contains("Build assertion {} saved to disk.".format(snap_build_path)),
        )
        self.assertThat(snap_build_path, FileExists())
        fake_check_output.mock.assert_called_with(
            [
                mock.ANY,
                "sign-build",
                "--developer-id=abcd",
                "--snap-id=snap-id",
                "--grade=stable",
                "-k",
                "default",
                self.snap_test.snap_path,
            ]
        )

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_missing_grade(
        self, mock_get_snap_data, mock_get_account_info,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "a_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap"}
        fake_check_output = fixtures.MockPatch(
            "subprocess.check_output",
            side_effect=['[{"name": "default"}]'.encode(), b"Mocked assertion"],
        )
        self.useFixture(fake_check_output)

        result = self.run_command(["sign-build", self.snap_test.snap_path, "--local"])

        self.assertThat(result.exit_code, Equals(0))
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(
            result.output,
            Contains("Build assertion {} saved to disk.".format(snap_build_path)),
        )
        self.assertThat(snap_build_path, FileExists())
        fake_check_output.mock.assert_called_with(
            [
                mock.ANY,
                "sign-build",
                "--developer-id=abcd",
                "--snap-id=snap-id",
                "--grade=stable",
                "-k",
                "default",
                self.snap_test.snap_path,
            ]
        )

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "push_snap_build")
    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_upload_successfully(
        self, mock_get_snap_data, mock_get_account_info, mock_push_snap_build,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "a_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        fake_check_output = fixtures.MockPatch(
            "subprocess.check_output",
            side_effect=[
                '[{"name": "default", "sha3-384": "a_hash"}]'.encode(),
                b"Mocked assertion",
            ],
        )
        self.useFixture(fake_check_output)

        result = self.run_command(["sign-build", self.snap_test.snap_path])

        self.assertThat(result.exit_code, Equals(0))
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(
            result.output,
            Contains("Build assertion {} saved to disk.".format(snap_build_path)),
        )
        self.assertThat(
            result.output,
            Contains(
                "Build assertion {} uploaded to the Store.".format(snap_build_path)
            ),
        )
        self.assertThat(snap_build_path, FileExists())
        fake_check_output.mock.assert_called_with(
            [
                mock.ANY,
                "sign-build",
                "--developer-id=abcd",
                "--snap-id=snap-id",
                "--grade=stable",
                "-k",
                "default",
                self.snap_test.snap_path,
            ]
        )
        mock_push_snap_build.assert_called_with("snap-id", "Mocked assertion")

    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "push_snap_build")
    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    def test_sign_build_upload_existing(
        self, mock_get_snap_data, mock_get_account_info, mock_push_snap_build,
    ):
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}

        snap_build_path = self.snap_test.snap_path + "-build"
        with open(snap_build_path, "wb") as fd:
            fd.write(b"Already signed assertion")

        result = self.run_command(["sign-build", self.snap_test.snap_path])

        self.assertThat(result.exit_code, Equals(0))
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(
            result.output,
            Contains("A signed build assertion for this snap already exists."),
        )
        self.assertThat(
            result.output,
            Contains(
                "Build assertion {} uploaded to the Store.".format(snap_build_path)
            ),
        )
        mock_push_snap_build.assert_called_with("snap-id", "Already signed assertion")
