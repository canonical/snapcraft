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


class BuildStateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.project = Project()
        self.property_names = ["foo"]
        self.part_properties = {"foo": "bar"}

        self.state = snapcraft.internal.states.BuildState(
            self.property_names, self.part_properties, self.project
        )


class BuildStateTestCase(BuildStateBaseTestCase):
    @mock.patch.object(
        snapcraft.internal.states.BuildState,
        "__init__",
        wraps=snapcraft.internal.states.BuildState.__init__,
    )
    def test_yaml_conversion(self, init_spy):
        state_string = yaml_utils.dump(self.state)

        # Verify that the dumped tag was correct
        self.assertThat(state_string.splitlines()[0], Equals("!BuildState"))

        # Now verify the conversion
        state_from_yaml = yaml_utils.load(state_string)
        self.assertThat(state_from_yaml, Equals(self.state))

        # Verify that init was not called
        init_spy.assert_not_called()

    def test_comparison(self):
        other = snapcraft.internal.states.BuildState(
            self.property_names, self.part_properties, self.project
        )

        self.assertTrue(self.state == other, "Expected states to be identical")

    def test_properties_of_interest(self):
        self.part_properties.update(
            {
                "after": "test-after",
                "build-attributes": ["test-build-attribute"],
                "build-packages": "test-build-packages",
                "disable-parallel": "test-disable-parallel",
                "organize": {"baz": "qux"},
                "override-build": "touch override-build",
            }
        )

        properties = self.state.properties_of_interest(self.part_properties)
        self.assertThat(len(properties), Equals(7))
        self.assertThat(properties["foo"], Equals("bar"))
        self.assertThat(properties["after"], Equals("test-after"))
        self.assertThat(
            properties["build-attributes"], Equals(["test-build-attribute"])
        )
        self.assertThat(properties["build-packages"], Equals("test-build-packages"))
        self.assertThat(properties["disable-parallel"], Equals("test-disable-parallel"))
        self.assertThat(properties["organize"], Equals({"baz": "qux"}))
        self.assertThat(properties["override-build"], Equals("touch override-build"))

    def test_project_options_of_interest(self):
        options = self.state.project_options_of_interest(self.project)

        self.assertThat(len(options), Equals(1))
        self.assertThat(options["deb_arch"], Equals("amd64"))


def test_comparison_not_equal(build_state, build_state_variant):
    assert build_state != build_state_variant
