# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import integration_tests


class StateMigrationTestCase(integration_tests.TestCase):

    def test_all_steps_not_dirty(self):
        project_dir = '2.22.1-prime-state'
        output = self.run_snapcraft('pull', project_dir)
        self.assertRegex(output, 'Skipping pull part-name')
        self.assertRegex(output, 'Skipping build part-name')
        self.assertRegex(output, 'Skipping stage part-name')
        self.assertRegex(output, 'Skipping prime part-name')
