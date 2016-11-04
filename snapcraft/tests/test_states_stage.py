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

import snapcraft.internal
from snapcraft import tests


class StageStateTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        class Project:
            pass

        self.project = Project()
        self.files = {'foo'}
        self.directories = {'bar'}
        self.part_properties = {'stage': ['baz']}

        self.state = snapcraft.internal.states.StageState(
            self.files, self.directories, self.part_properties, self.project)

    def test_representation(self):
        expected = ('StageState(directories: {}, files: {}, '
                    'project_options: {}, properties: {})').format(
            self.directories, self.files, self.project.__dict__,
            self.part_properties)
        self.assertEqual(expected, repr(self.state))

    def test_comparison(self):
        other = snapcraft.internal.states.StageState(
            self.files, self.directories, self.part_properties, self.project)

        self.assertTrue(self.state == other, 'Expected states to be identical')

    def test_comparison_not_equal(self):
        others = [
            snapcraft.internal.states.StageState(
                set(), self.directories, self.part_properties, self.project),
            snapcraft.internal.states.StageState(
                self.files, set(), self.part_properties, self.project),
            snapcraft.internal.states.StageState(
                self.files, self.directories, None, self.project),
        ]

        for index, other in enumerate(others):
            with self.subTest('other #{}'.format(index+1)):
                self.assertFalse(self.state == other,
                                 'Expected states to be different')
