# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Dan Chapman <dpniel@ubuntu.com>
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

import testscenarios
import integration_tests


class QbsPluginTestCase(testscenarios.WithScenarios,
                        integration_tests.TestCase):

    scenarios = [
        # we expect qbs-nil to output the wrong string here as
        # 'project.printString' is set to false in the yaml file.
        ('qbs-nil', dict(project_dir='qbs-nil',
                         expected_value='Wrong snapcraft string')),
        ('qbs-qt', dict(project_dir='qbs-qt',
                        expected_value='Hello Snapcraft World')),
    ]

    def test_stage_qbs_plugin(self):

        self.run_snapcraft('stage', self.project_dir)

        # The test binaries have the same name as the project dir
        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', self.project_dir))
        self.assertEqual(self.expected_value, binary_output)
