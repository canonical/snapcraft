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

import yaml

import snapcraft.internal
from snapcraft import tests


class BuildStateTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        class Project:
            def __init__(self):
                self.deb_arch = 'amd64'

        self.project = Project()
        self.property_names = ['foo']
        self.part_properties = {'foo': 'bar'}

        self.state = snapcraft.internal.states.BuildState(
            self.property_names, self.part_properties, self.project)

    def test_yaml_conversion(self):
        state_from_yaml = yaml.load(yaml.dump(self.state))
        self.assertEqual(self.state, state_from_yaml)

    def test_comparison(self):
        other = snapcraft.internal.states.BuildState(
            self.property_names, self.part_properties, self.project)

        self.assertTrue(self.state == other, 'Expected states to be identical')

    def test_comparison_not_equal(self):
        others = [
            snapcraft.internal.states.BuildState(
                [], self.part_properties, self.project),
            snapcraft.internal.states.BuildState(
                self.property_names, None, self.project),
            snapcraft.internal.states.BuildState(
                self.property_names, self.part_properties, None)
        ]

        for index, other in enumerate(others):
            with self.subTest('other #{}'.format(index+1)):
                self.assertFalse(self.state == other,
                                 'Expected states to be different')

    def test_properties_of_interest(self):
        self.part_properties.update({
            'after': 'test-after',
            'build-packages': 'test-build-packages',
            'disable-parallel': 'test-disable-parallel',
            'organize': {'baz': 'qux'}
        })

        properties = self.state.properties_of_interest(self.part_properties)
        self.assertEqual(5, len(properties))
        self.assertEqual('bar', properties['foo'])
        self.assertEqual('test-after', properties['after'])
        self.assertEqual('test-build-packages', properties['build-packages'])
        self.assertEqual('test-disable-parallel',
                         properties['disable-parallel'])
        self.assertEqual({'baz': 'qux'}, properties['organize'])

    def test_project_options_of_interest(self):
        options = self.state.project_options_of_interest(self.project)

        self.assertEqual(1, len(options))
        self.assertEqual('amd64', options['deb_arch'])
