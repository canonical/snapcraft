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

import snapcraft.internal
from snapcraft import yaml_utils
from tests import unit

from .conftest import Project


class StageStateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

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
    @mock.patch.object(
        snapcraft.internal.states.StageState,
        "__init__",
        wraps=snapcraft.internal.states.StageState.__init__,
    )
    def test_yaml_conversion(self, init_spy):
        state_string = yaml_utils.dump(self.state)

        # Verify that the dumped tag was correct
        self.assertThat(state_string.splitlines()[0], Equals("!StageState"))

        # Now verify the conversion
        state_from_yaml = yaml_utils.load(state_string)
        self.assertThat(state_from_yaml, Equals(self.state))

        # Verify that init was not called
        init_spy.assert_not_called()

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


def test_comparison_not_equal(stage_state, stage_state_variant):
    assert stage_state != stage_state_variant
