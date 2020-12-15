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


class PullStateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.project = Project()
        self.property_names = ["foo"]
        self.part_properties = {"foo": "bar"}

        self.state = snapcraft.internal.states.PullState(
            self.property_names, self.part_properties, self.project
        )


class PullStateTestCase(PullStateBaseTestCase):
    @mock.patch.object(
        snapcraft.internal.states.PullState,
        "__init__",
        wraps=snapcraft.internal.states.PullState.__init__,
    )
    def test_yaml_conversion(self, init_spy):
        state_string = yaml_utils.dump(self.state)

        # Verify that the dumped tag was correct
        self.assertThat(state_string.splitlines()[0], Equals("!PullState"))

        # Now verify the conversion
        state_from_yaml = yaml_utils.load(state_string)
        self.assertThat(state_from_yaml, Equals(self.state))

        # Verify that init was not called
        init_spy.assert_not_called()

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


def test_comparison_not_equal(pull_state, pull_state_variant):
    assert pull_state != pull_state_variant
