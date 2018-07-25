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

from textwrap import dedent
from unittest import mock

from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from . import CommandBaseTestCase, get_sample_key, mock_snap_output


class ListKeysCommandTestCase(CommandBaseTestCase):

    scenarios = [
        ("list-keys", {"command_name": "list-keys"}),
        ("keys alias", {"command_name": "keys"}),
    ]

    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_list_keys_snapd_not_installed(self, mock_installed, mock_check_output):
        mock_installed.return_value = False

        raised = self.assertRaises(
            storeapi.errors.MissingSnapdError, self.run_command, [self.command_name]
        )

        self.assertThat(str(raised), Contains("The snapd package is not installed."))
        mock_installed.assert_called_with("snapd")
        self.assertThat(mock_check_output.call_count, Equals(0))

    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_list_keys_without_login(self, mock_installed, mock_check_output):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output

        raised = self.assertRaises(
            storeapi.errors.InvalidCredentialsError,
            self.run_command,
            [self.command_name],
        )

        self.assertThat(str(raised), Contains("Invalid credentials"))

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_list_keys_successfully(
        self, mock_installed, mock_check_output, mock_get_account_information
    ):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {
            "account_id": "abcd",
            "account_keys": [
                {
                    "name": "default",
                    "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                }
            ],
        }

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
                Name     SHA3-384 fingerprint
            *   default  {default_sha3_384}
            -   another  {another_sha3_384}  (not registered)
            """
                ).format(
                    default_sha3_384=get_sample_key("default")["sha3-384"],
                    another_sha3_384=get_sample_key("another")["sha3-384"],
                )
            ),
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    @mock.patch("subprocess.check_output")
    @mock.patch("snapcraft.internal.repo.Repo.is_package_installed")
    def test_list_keys_without_registered(
        self, mock_installed, mock_check_output, mock_get_account_information
    ):
        mock_installed.return_value = True
        mock_check_output.side_effect = mock_snap_output
        mock_get_account_information.return_value = {
            "account_id": "abcd",
            "account_keys": [],
        }

        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                "No keys have been registered. "
                "See 'snapcraft register-key --help' to register a key."
            ),
        )
