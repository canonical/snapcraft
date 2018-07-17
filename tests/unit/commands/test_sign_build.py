# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from snapcraft import storeapi
import tests
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

    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_snapd_not_installed(self, mock_installed, mock_check_output):
        mock_installed.return_value = False

        raised = self.assertRaises(
            storeapi.errors.MissingSnapdError,
            self.run_command,
            ["sign-build", self.snap_test.snap_path, "--local"],
        )

        self.assertThat(str(raised), Contains("The snapd package is not installed."))
        mock_installed.assert_called_with("snapd")
        self.assertThat(mock_check_output.call_count, Equals(0))

    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_nonexisting_snap(self, mock_installed, mock_check_output):
        mock_installed.return_value = True

        result = self.run_command(["sign-build", "nonexisting.snap"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output, Contains('Path "nonexisting.snap" does not exist')
        )
        self.assertThat(mock_check_output.call_count, Equals(0))

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_missing_account_info(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
    ):
        mock_installed.return_value = True
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
        self.assertThat(mock_check_output.call_count, Equals(0))

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_no_usable_keys(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
    ):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        mock_check_output.side_effect = ["[]"]

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

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_no_usable_named_key(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
    ):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        mock_check_output.side_effect = ['[{"name": "default"}]']

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

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_unregistered_key(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
    ):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "another_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        mock_check_output.side_effect = ['[{"name": "default", "sha3-384": "a_hash"}]']

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
                "before signing and pushing signatures to the Store."
            ),
        )
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(snap_build_path, Not(FileExists()))

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_snapd_failure(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
    ):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "a_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        mock_check_output.side_effect = [
            '[{"name": "default", "sha3-384": "a_hash"}]',
            subprocess.CalledProcessError(1, ["a", "b"]),
        ]

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

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_locally_successfully(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
    ):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        mock_check_output.side_effect = ['[{"name": "default"}]', b"Mocked assertion"]

        result = self.run_command(["sign-build", self.snap_test.snap_path, "--local"])

        self.assertThat(result.exit_code, Equals(0))
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(
            result.output,
            Contains("Build assertion {} saved to disk.".format(snap_build_path)),
        )
        self.assertThat(snap_build_path, FileExists())
        mock_check_output.assert_called_with(
            [
                "snap",
                "sign-build",
                "--developer-id=abcd",
                "--snap-id=snap-id",
                "--grade=stable",
                "-k",
                "default",
                self.snap_test.snap_path,
            ]
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_missing_grade(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
    ):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "a_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap"}
        mock_check_output.side_effect = [
            '[{"name": "default", "sha3-384": "a_hash"}]',
            b"Mocked assertion",
        ]

        result = self.run_command(["sign-build", self.snap_test.snap_path, "--local"])

        self.assertThat(result.exit_code, Equals(0))
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(
            result.output,
            Contains("Build assertion {} saved to disk.".format(snap_build_path)),
        )
        self.assertThat(snap_build_path, FileExists())
        mock_check_output.assert_called_with(
            [
                "snap",
                "sign-build",
                "--developer-id=abcd",
                "--snap-id=snap-id",
                "--grade=stable",
                "-k",
                "default",
                self.snap_test.snap_path,
            ]
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "push_snap_build")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_push_successfully(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_check_output,
        mock_get_account_info,
        mock_push_snap_build,
    ):
        mock_installed.return_value = True
        mock_get_account_info.return_value = {
            "account_id": "abcd",
            "account_keys": [{"public-key-sha3-384": "a_hash"}],
            "snaps": {"16": {"test-snap": {"snap-id": "snap-id"}}},
        }
        mock_get_snap_data.return_value = {"name": "test-snap", "grade": "stable"}
        mock_check_output.side_effect = [
            '[{"name": "default", "sha3-384": "a_hash"}]',
            b"Mocked assertion",
        ]

        result = self.run_command(["sign-build", self.snap_test.snap_path])

        self.assertThat(result.exit_code, Equals(0))
        snap_build_path = self.snap_test.snap_path + "-build"
        self.assertThat(
            result.output,
            Contains("Build assertion {} saved to disk.".format(snap_build_path)),
        )
        self.assertThat(
            result.output,
            Contains("Build assertion {} pushed to the Store.".format(snap_build_path)),
        )
        self.assertThat(snap_build_path, FileExists())
        mock_check_output.assert_called_with(
            [
                "snap",
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

    @mock.patch.object(storeapi._sca_client.SCAClient, "push_snap_build")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("snapcraft._store._get_data_from_snap_file")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_sign_build_push_existing(
        self,
        mock_installed,
        mock_get_snap_data,
        mock_get_account_info,
        mock_push_snap_build,
    ):
        mock_installed.return_value = True
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
            Contains("Build assertion {} pushed to the Store.".format(snap_build_path)),
        )
        mock_push_snap_build.assert_called_with("snap-id", "Already signed assertion")
