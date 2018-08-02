# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import os
import yaml

from snapcraft.internal import steps


class State(yaml.YAMLObject):
    def __repr__(self):
        items = sorted(self.__dict__.items())
        strings = (": ".join((key, repr(value))) for key, value in items)
        representation = ", ".join(strings)

        return "{}({})".format(self.__class__.__name__, representation)

    def __eq__(self, other):
        if type(other) is type(self):
            return self.__dict__ == other.__dict__

        return False


class PartState(State):
    def __init__(self, part_properties, project):
        super().__init__()
        if not part_properties:
            part_properties = {}

        self.properties = self.properties_of_interest(part_properties)
        self.project_options = self.project_options_of_interest(project)

    def properties_of_interest(self, part_properties):
        """Extract the properties concerning this step from the options.

        Note that these options come from the YAML for a given part.
        """

        raise NotImplementedError

    def project_options_of_interest(self, project):
        """Extract the options concerning this step from the project."""

        raise NotImplementedError

    def diff_properties_of_interest(self, other_properties):
        """Return set of properties that differ."""

        return _get_differing_keys(
            self.properties, self.properties_of_interest(other_properties)
        )

    def diff_project_options_of_interest(self, other_project_options):
        """Return set of project options that differ."""

        return _get_differing_keys(
            self.project_options,
            self.project_options_of_interest(other_project_options),
        )


def _get_differing_keys(dict1, dict2):
    differing_keys = set()
    for key, dict1_value in dict1.items():
        dict2_value = dict2.get(key)
        if dict1_value != dict2_value:
            differing_keys.add(key)

    for key, dict2_value in dict2.items():
        dict1_value = dict1.get(key)
        if dict1_value != dict2_value:
            differing_keys.add(key)

    return differing_keys


def get_state(state_dir: str, step: steps.Step):
    state = None
    state_file = get_step_state_file(state_dir, step)
    if os.path.isfile(state_file):
        with open(state_file, "r") as f:
            state = yaml.load(f.read())

    return state


def get_step_state_file(state_dir: str, step: steps.Step) -> str:
    return os.path.join(state_dir, step.name)
