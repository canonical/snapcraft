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

import subprocess

from testtools.matchers import (
    Contains,
    Equals,
    Not,
)

from tests import integration


class PullTestCase(integration.TestCase):

    def _pull_invalid_part(self, debug):
        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, ['pull', 'invalid-part-name'],
            'go-hello', debug=debug)

        self.assertThat(exception.returncode, Equals(2))
        self.assertThat(exception.output, Contains(
            "part named 'invalid-part-name' is not defined"))

        return exception.output

    def test_pull_invalid_part_no_traceback_without_debug(self):
        self.assertThat(
            self._pull_invalid_part(False), Not(Contains("Traceback")))

    def test_pull_invalid_part_does_traceback_with_debug(self):
        self.assertThat(
            self._pull_invalid_part(True), Contains("Traceback"))
