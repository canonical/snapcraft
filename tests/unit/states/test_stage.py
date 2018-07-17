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

import yaml

from testtools.matchers import Equals

import snapcraft.internal
from tests import unit


class StageStateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Project:
            pass

        self.project = Project()
        self.files = {"foo"}
        self.directories = {"bar"}
        self.part_properties = {
            "filesets": {"qux": "quux"},
            "override-stage": "touch override-stage",
            "stage": ["baz"],
        }

        self.state = snapcraft.internal.states.StageState(
            self.files, self.directories, self.part_properties, self.project
        )


class StateStageTestCase(StageStateBaseTestCase):
    def test_yaml_conversion(self):
        state_from_yaml = yaml.load(yaml.dump(self.state))
        self.assertThat(state_from_yaml, Equals(self.state))

    def test_comparison(self):
        other = snapcraft.internal.states.StageState(
            self.files, self.directories, self.part_properties, self.project
        )

        self.assertTrue(self.state == other, "Expected states to be identical")

    def test_properties_of_interest(self):
        properties = self.state.properties_of_interest(self.part_properties)
        self.assertThat(len(properties), Equals(3))
        self.assertThat(properties["filesets"], Equals({"qux": "quux"}))
        self.assertThat(properties["override-stage"], Equals("touch override-stage"))
        self.assertThat(properties["stage"], Equals(["baz"]))

    def test_project_options_of_interest(self):
        self.assertFalse(self.state.project_options_of_interest(self.project))


class StageStateNotEqualTestCase(StageStateBaseTestCase):

    scenarios = [
        ("no files", dict(other_property="files", other_value=set())),
        ("no directories", dict(other_property="directories", other_value=set())),
        (
            "no part properties",
            dict(other_property="part_properties", other_value=None),
        ),
    ]

    def test_comparison_not_equal(self):
        setattr(self, self.other_property, self.other_value)
        other_state = snapcraft.internal.states.StageState(
            self.files, self.directories, self.part_properties, self.project
        )

        self.assertFalse(self.state == other_state, "Expected states to be different")
