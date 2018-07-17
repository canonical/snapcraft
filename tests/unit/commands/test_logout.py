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
import re
from unittest import mock

from testtools.matchers import MatchesRegex, Equals

from snapcraft import config
from . import CommandBaseTestCase


class LogoutCommandTestCase(CommandBaseTestCase):
    @mock.patch.object(config.Config, "clear")
    def test_logout_clears_config(self, mock_clear):
        result = self.run_command(["logout"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, MatchesRegex(".*Credentials cleared.\n", flags=re.DOTALL)
        )
