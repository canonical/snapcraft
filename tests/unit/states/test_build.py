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


class BuildStateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Project:
            def __init__(self):
                self.deb_arch = "amd64"

        self.project = Project()
        self.property_names = ["foo"]
        self.part_properties = {"foo": "bar"}

        self.state = snapcraft.internal.states.BuildState(
            self.property_names, self.part_properties, self.project
        )


class BuildStateTestCase(BuildStateBaseTestCase):
    def test_yaml_conversion(self):
        state_from_yaml = yaml.load(yaml.dump(self.state))
        self.assertThat(state_from_yaml, Equals(self.state))

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
                "prepare": "touch prepare",
                "build": "touch build",
                "install": "touch install",
            }
        )

        properties = self.state.properties_of_interest(self.part_properties)
        self.assertThat(len(properties), Equals(10))
        self.assertThat(properties["foo"], Equals("bar"))
        self.assertThat(properties["after"], Equals("test-after"))
        self.assertThat(
            properties["build-attributes"], Equals(["test-build-attribute"])
        )
        self.assertThat(properties["build-packages"], Equals("test-build-packages"))
        self.assertThat(properties["disable-parallel"], Equals("test-disable-parallel"))
        self.assertThat(properties["organize"], Equals({"baz": "qux"}))
        self.assertThat(properties["override-build"], Equals("touch override-build"))
        self.assertThat(properties["prepare"], Equals("touch prepare"))
        self.assertThat(properties["build"], Equals("touch build"))
        self.assertThat(properties["install"], Equals("touch install"))

    def test_project_options_of_interest(self):
        options = self.state.project_options_of_interest(self.project)

        self.assertThat(len(options), Equals(1))
        self.assertThat(options["deb_arch"], Equals("amd64"))


class BuildStateNotEqualTestCase(BuildStateBaseTestCase):

    scenarios = [
        ("no property names", dict(other_property="property_names", other_value=[])),
        (
            "no part properties",
            dict(other_property="part_properties", other_value=None),
        ),
        ("no project", dict(other_property="project", other_value=None)),
    ]

    def test_comparison_not_equal(self):
        setattr(self, self.other_property, self.other_value)
        other_state = snapcraft.internal.states.BuildState(
            self.property_names, self.part_properties, self.project
        )
        self.assertFalse(self.state == other_state, "Expected states to be different")
