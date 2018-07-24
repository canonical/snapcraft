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

from snapcraft.integrations import travis
from . import CommandBaseTestCase


class EnableCITestCase(CommandBaseTestCase):
    def test_enable_ci_empty(self):
        result = self.run_command(["enable-ci"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output,
            Contains('Missing argument "ci-system".  Choose from travis.'),
        )

    def test_enable_ci_unknown(self):
        result = self.run_command(["enable-ci", "bazinga"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output, Contains("invalid choice: bazinga. (choose from travis)")
        )

    @mock.patch.object(travis, "__doc__")
    @mock.patch.object(travis, "enable")
    def test_enable_ci_travis(self, mock_enable, mock_doc):
        self.make_snapcraft_yaml("name: foo")

        mock_doc.__str__.return_value = "<module docstring>"

        result = self.run_command(["enable-ci", "travis"], input="y\n")

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("<module docstring>"))
        self.assertThat(mock_enable.call_count, Equals(1))

    @mock.patch.object(travis, "refresh")
    def test_enable_ci_travis_refresh(self, mock_refresh):
        self.make_snapcraft_yaml("name: foo")

        result = self.run_command(["enable-ci", "travis", "--refresh"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Equals(""))
        self.assertThat(mock_refresh.call_count, Equals(1))
