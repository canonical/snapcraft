# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

from testtools.matchers import EndsWith

import integration_tests


class StageTestCase(integration_tests.TestCase):

    def test_conflicts(self):
        project_dir = 'conflicts'
        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'stage', project_dir)

        self.assertEqual(1, exception.returncode)
        expected = (
            "Parts 'p1' and 'p2' have the following file paths in common "
            "which have different contents:\nbin/test\n")
        self.assertThat(exception.output, EndsWith(expected))
