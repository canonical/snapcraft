# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from unittest import mock
from testtools.matchers import Equals

from snapcraft import yaml_utils
import snapcraft.internal
from tests import unit


class PrimeStateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Project:
            pass

        self.project = Project()
        self.files = {"foo"}
        self.directories = {"bar"}
        self.dependency_paths = {"baz"}
        self.part_properties = {
            "override-prime": "touch override-prime",
            "prime": ["qux"],
        }

        self.state = snapcraft.internal.states.PrimeState(
            self.files,
            self.directories,
            self.dependency_paths,
            self.part_properties,
            self.project,
        )


class PrimeStateTestCase(PrimeStateBaseTestCase):
    @mock.patch.object(
        snapcraft.internal.states.PrimeState,
        "__init__",
        wraps=snapcraft.internal.states.PrimeState.__init__,
    )
    def test_yaml_conversion(self, init_spy):
        state_string = yaml_utils.dump(self.state)

        # Verify that the dumped tag was correct
        self.assertThat(state_string.splitlines()[0], Equals("!PrimeState"))

        # Now verify the conversion
        state_from_yaml = yaml_utils.load(state_string)
        self.assertThat(state_from_yaml, Equals(self.state))

        # Verify that init was not called
        init_spy.assert_not_called()

    def test_comparison(self):
        other = snapcraft.internal.states.PrimeState(
            self.files,
            self.directories,
            self.dependency_paths,
            self.part_properties,
            self.project,
        )

        self.assertTrue(self.state == other, "Expected states to be identical")

    def test_properties_of_interest(self):
        properties = self.state.properties_of_interest(self.part_properties)
        self.assertThat(len(properties), Equals(2))
        self.assertThat(properties["override-prime"], Equals("touch override-prime"))
        self.assertThat(properties["prime"], Equals(["qux"]))

    def test_project_options_of_interest(self):
        self.assertFalse(self.state.project_options_of_interest(self.project))


class PrimeStateNotEqualTestCase(PrimeStateBaseTestCase):

    scenarios = [
        ("no files", dict(other_property="files", other_value=set())),
        ("no directories", dict(other_property="directories", other_value=set())),
        (
            "no dependency paths",
            dict(other_property="dependency_paths", other_value=set()),
        ),
        (
            "no part properties",
            dict(other_property="part_properties", other_value=None),
        ),
    ]

    def test_comparison_not_equal(self):
        setattr(self, self.other_property, self.other_value)
        other_state = snapcraft.internal.states.PrimeState(
            self.files,
            self.directories,
            self.dependency_paths,
            self.part_properties,
            self.project,
        )

        self.assertFalse(self.state == other_state, "Expected states to be different")
