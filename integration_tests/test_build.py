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

import subprocess

from testtools.matchers import (
    Contains,
    Equals,
    Not,
)

import integration_tests


class BuildTestCase(integration_tests.TestCase):

    def _build_invalid_part(self, debug):
        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['build', 'invalid-part-name'],
            'go-hello', debug=debug)

        self.assertThat(exception.returncode, Equals(2))
        self.assertThat(exception.output, Contains(
            "part named 'invalid-part-name' is not defined"))

        return exception.output

    def test_build_invalid_part_no_traceback_without_debug(self):
        self.assertThat(
            self._build_invalid_part(False), Not(Contains("Traceback")))

    def test_build_invalid_part_does_traceback_with_debug(self):
        self.assertThat(
            self._build_invalid_part(True), Contains("Traceback"))
