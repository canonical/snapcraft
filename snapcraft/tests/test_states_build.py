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


class BuildStateTestCase(tests.TestCase):
    def setUp(self):
        super().setUp()

        self.schema_properties = ['foo']

        class Options:
            def __init__(self):
                self.foo = ['bar']

        self.options = Options()

        self.state = snapcraft.internal.states.BuildState(
            self.schema_properties, self.options)

    def test_representation(self):
        expected = 'BuildState(properties: {}, schema_properties: {})'.format(
            self.options.__dict__, self.schema_properties)
        self.assertEqual(expected, repr(self.state))

    def test_comparison(self):
        other = snapcraft.internal.states.BuildState(
            self.schema_properties, self.options)

        self.assertTrue(self.state == other, 'Expected states to be identical')

    def test_comparison_not_equal(self):
        others = [
            snapcraft.internal.states.BuildState([], self.options),
            snapcraft.internal.states.BuildState(self.schema_properties, None)
        ]

        for index, other in enumerate(others):
            with self.subTest('other #{}'.format(index+1)):
                self.assertFalse(self.state == other,
                                 'Expected states to be different')
