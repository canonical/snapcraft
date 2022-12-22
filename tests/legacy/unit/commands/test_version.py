# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
from testtools.matchers import Equals

from . import CommandBaseTestCase


class VersionCommandTestCase(CommandBaseTestCase):
    def test_has_version(self):
        result = self.run_command(["--version"])
        self.assertThat(result.exit_code, Equals(0))

    def test_has_version_without_hyphens(self):
        result = self.run_command(["version"])
        self.assertThat(result.exit_code, Equals(0))

    def test_method_return_same_value(self):
        result1 = self.run_command(["version"])
        result2 = self.run_command(["--version"])
        self.assertEqual(result1.output, result2.output)
