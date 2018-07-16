# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
import re
import xdg

from unittest import mock

import fixtures
from testtools.matchers import MatchesRegex

from snapcraft import storeapi
from tests.unit import commands


class WhoamiCommandBaseTestCase(commands.CommandBaseTestCase):
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    def test_unknown_email_must_suggest_logout_and_login(
        self, mock_get_account_information
    ):
        mock_get_account_information.return_value = {}
        self.useFixture(fixtures.EnvironmentVariable("HOME", self.path))

        config_file_path = os.path.join(
            xdg.BaseDirectory.save_config_path("snapcraft"), "snapcraft.cfg"
        )
        with open(config_file_path, "w") as config_file:
            config_file.write("[login.ubuntu.com]\n")
            config_file.write("account_id = test_account_id\n")

        result = self.run_command(["whoami"])
        self.assertThat(
            result.output,
            MatchesRegex(
                ".*email: +unknown\n"
                "developer-id: +test_account_id\n"
                ".*logout and login again.*",
                flags=re.DOTALL,
            ),
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    def test_whoami_must_print_email_and_developer_id(
        self, mock_get_account_information
    ):
        mock_get_account_information.return_value = {}
        self.useFixture(fixtures.EnvironmentVariable("HOME", self.path))
        config_file_path = os.path.join(
            xdg.BaseDirectory.save_config_path("snapcraft"), "snapcraft.cfg"
        )
        with open(config_file_path, "w") as config_file:
            config_file.write("[login.ubuntu.com]\n")
            config_file.write("email = test@example.com\n")
            config_file.write("account_id = test_account_id\n")

        result = self.run_command(["whoami"])
        self.assertThat(
            result.output,
            MatchesRegex(
                ".*email: +test@example.com\ndeveloper-id: +test_account_id\n",
                flags=re.DOTALL,
            ),
        )
