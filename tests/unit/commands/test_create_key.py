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
from unittest import mock

from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from . import CommandBaseTestCase, get_sample_key, mock_snap_output


class CreateKeyTestCase(CommandBaseTestCase):
    @mock.patch("subprocess.check_call")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_create_key_snapd_not_installed(
        self, mock_installed, mock_check_output, mock_check_call
    ):
        mock_installed.return_value = False

        raised = self.assertRaises(
            storeapi.errors.MissingSnapdError, self.run_command, ["create-key"]
        )

        self.assertThat(str(raised), Contains("The snapd package is not installed."))

        mock_installed.assert_called_with("snapd")
        self.assertThat(mock_check_output.call_count, Equals(0))
        self.assertThat(mock_check_call.call_count, Equals(0))

    @mock.patch("subprocess.check_call")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_create_key_already_exists(
        self, mock_installed, mock_check_output, mock_check_call
    ):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output

        raised = self.assertRaises(
            storeapi.errors.KeyAlreadyRegisteredError, self.run_command, ["create-key"]
        )

        self.assertThat(
            str(raised), Equals("You have already registered a key named 'default'")
        )

    @mock.patch("subprocess.check_call")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_create_key_already_registered(
        self,
        mock_installed,
        mock_check_output,
        mock_get_account_information,
        mock_check_call,
    ):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {
            "account_id": "abcd",
            "account_keys": [
                {
                    "name": "new-key",
                    "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                }
            ],
        }

        raised = self.assertRaises(
            storeapi.errors.KeyAlreadyRegisteredError,
            self.run_command,
            ["create-key", "new-key"],
        )

        self.assertThat(
            str(raised), Equals("You have already registered a key named 'new-key'")
        )
        self.assertThat(mock_check_call.call_count, Equals(0))

    @mock.patch("subprocess.check_call")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_create_key_successfully(
        self,
        mock_installed,
        mock_check_output,
        mock_get_account_information,
        mock_check_call,
    ):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {
            "account_id": "abcd",
            "account_keys": [
                {
                    "name": "old-key",
                    "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                }
            ],
        }

        result = self.run_command(["create-key", "new-key"])

        self.assertThat(result.exit_code, Equals(0))
        mock_check_call.assert_called_once_with(["snap", "create-key", "new-key"])

    @mock.patch("subprocess.check_call")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_create_key_without_login(
        self,
        mock_installed,
        mock_check_output,
        mock_get_account_information,
        mock_check_call,
    ):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.side_effect = storeapi.errors.InvalidCredentialsError(
            "Test"
        )

        result = self.run_command(["create-key", "new-key"])

        self.assertThat(result.exit_code, Equals(0))

        mock_check_call.assert_called_once_with(["snap", "create-key", "new-key"])
