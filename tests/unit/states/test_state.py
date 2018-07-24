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

from testtools.matchers import Equals

from snapcraft.internal.states._state import PartState
from tests import unit


class _TestState(PartState):
    def properties_of_interest(self, part_properties):
        return {"foo": part_properties.get("foo"), "baz": part_properties.get("baz")}

    def project_options_of_interest(self, project):
        return {
            "foo": getattr(project, "foo", None),
            "baz": getattr(project, "baz", None),
        }


class _TestProject:
    def __init__(self, dict):
        for key, value in dict.items():
            setattr(self, key, value)


class StateTestCase(unit.TestCase):
    scenarios = [
        ("change existing", dict(old={"foo": "bar"}, new={"foo": "baz"})),
        ("add new", dict(old={"baz": "qux"}, new={"foo": "bar", "baz": "qux"})),
        ("remove old", dict(old={"foo": "bar", "baz": "qux"}, new={"baz": "qux"})),
    ]

    def setUp(self):
        super().setUp()

        self.project = _TestProject(self.old)
        self.part_properties = self.old

        self.state = _TestState(self.part_properties, self.project)
        self.state.properties = self.old
        self.state.project_options = self.old

    def test_diff_properties_of_interest(self):
        differing_properties = self.state.diff_properties_of_interest(self.new)
        self.assertThat(differing_properties, Equals({"foo"}))

    def test_diff_project_options_of_interest(self):
        differing_properties = self.state.diff_project_options_of_interest(
            _TestProject(self.new)
        )
        self.assertThat(differing_properties, Equals({"foo"}))
