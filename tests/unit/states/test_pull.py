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


class PullStateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Project:
            def __init__(self):
                self.deb_arch = "amd64"

        self.project = Project()
        self.property_names = ["foo"]
        self.part_properties = {"foo": "bar"}

        self.state = snapcraft.internal.states.PullState(
            self.property_names, self.part_properties, self.project
        )


class PullStateTestCase(PullStateBaseTestCase):
    def test_yaml_conversion(self):
        state_from_yaml = yaml.load(yaml.dump(self.state))
        self.assertThat(state_from_yaml, Equals(self.state))

    def test_comparison(self):
        other = snapcraft.internal.states.PullState(
            self.property_names, self.part_properties, self.project
        )

        self.assertTrue(self.state == other, "Expected states to be identical")

    def test_properties_of_interest(self):
        self.part_properties.update(
            {
                "override-pull": "touch override-pull",
                "plugin": "test-plugin",
                "parse-info": "test-parse-info",
                "stage-packages": ["test-stage-package"],
                "source": "test-source",
                "source-commit": "test-source-commit",
                "source-depth": "test-source-depth",
                "source-tag": "test-source-tag",
                "source-type": "test-source-type",
                "source-branch": "test-source-branch",
                "source-subdir": "test-source-subdir",
            }
        )

        properties = self.state.properties_of_interest(self.part_properties)
        self.assertThat(len(properties), Equals(12))
        self.assertThat(properties["foo"], Equals("bar"))
        self.assertThat(properties["override-pull"], Equals("touch override-pull"))
        self.assertThat(properties["plugin"], Equals("test-plugin"))
        self.assertThat(properties["parse-info"], Equals("test-parse-info"))
        self.assertThat(properties["stage-packages"], Equals(["test-stage-package"]))
        self.assertThat(properties["source"], Equals("test-source"))
        self.assertThat(properties["source-commit"], Equals("test-source-commit"))
        self.assertThat(properties["source-depth"], Equals("test-source-depth"))
        self.assertThat(properties["source-tag"], Equals("test-source-tag"))
        self.assertThat(properties["source-type"], Equals("test-source-type"))
        self.assertThat(properties["source-branch"], Equals("test-source-branch"))
        self.assertThat(properties["source-subdir"], Equals("test-source-subdir"))

    def test_project_options_of_interest(self):
        options = self.state.project_options_of_interest(self.project)

        self.assertThat(len(options), Equals(1))
        self.assertThat(options["deb_arch"], Equals("amd64"))


class PullStateNotEqualTestCase(PullStateBaseTestCase):

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
        other_state = snapcraft.internal.states.PullState(
            self.property_names, self.part_properties, self.project
        )

        self.assertFalse(self.state == other_state, "Expected states to be different")
