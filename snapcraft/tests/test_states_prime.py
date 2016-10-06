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


class PrimeStateTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        class Project:
            pass

        self.project = Project()
        self.files = {'foo'}
        self.directories = {'bar'}
        self.dependency_paths = {'baz'}
        self.part_properties = {'snap': ['gux']}

        self.state = snapcraft.internal.states.PrimeState(
            self.files, self.directories, self.dependency_paths,
            self.part_properties, self.project)

    def test_representation(self):
        expected = ('PrimeState(dependency_paths: {}, directories: {}, '
                    'files: {}, project_options: {}, properties: {})').format(
            self.dependency_paths, self.directories, self.files,
            self.project.__dict__, self.part_properties)
        self.assertEqual(expected, repr(self.state))

    def test_comparison(self):
        other = snapcraft.internal.states.PrimeState(
            self.files, self.directories, self.dependency_paths,
            self.part_properties, self.project)

        self.assertTrue(self.state == other, 'Expected states to be identical')

    def test_comparison_not_equal(self):
        others = [
            snapcraft.internal.states.PrimeState(
                set(), self.directories, self.dependency_paths,
                self.part_properties, self.project),
            snapcraft.internal.states.PrimeState(
                self.files, set(), self.dependency_paths,
                self.part_properties, self.project),
            snapcraft.internal.states.PrimeState(
                self.files, self.directories, set(),
                self.part_properties, self.project),
            snapcraft.internal.states.PrimeState(
                self.files, self.directories, self.dependency_paths,
                None, self.project)
        ]

        for index, other in enumerate(others):
            with self.subTest('other #{}'.format(index+1)):
                self.assertFalse(self.state == other,
                                 'Expected states to be different')
