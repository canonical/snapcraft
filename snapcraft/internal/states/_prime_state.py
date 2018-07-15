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

import yaml

import snapcraft.extractors
from snapcraft.internal.states._state import PartState


def _prime_state_constructor(loader, node):
    parameters = loader.construct_mapping(node)
    return PrimeState(**parameters)


yaml.add_constructor(u"!PrimeState", _prime_state_constructor)


class PrimeState(PartState):
    yaml_tag = u"!PrimeState"

    def __init__(
        self,
        files,
        directories,
        dependency_paths=None,
        part_properties=None,
        project=None,
        scriptlet_metadata=None,
    ):
        super().__init__(part_properties, project)

        if not scriptlet_metadata:
            scriptlet_metadata = snapcraft.extractors.ExtractedMetadata()

        self.files = files
        self.directories = directories
        self.dependency_paths = set()
        self.scriptlet_metadata = scriptlet_metadata

        if dependency_paths:
            self.dependency_paths = dependency_paths

    def properties_of_interest(self, part_properties):
        """Extract the properties concerning this step from part_properties.

        The only property of interest to the prime step is the `prime` keyword
        used to filter out files with a white or blacklist.
        """

        return {
            "override-prime": part_properties.get("override-prime"),
            "prime": part_properties.get("prime", ["*"]) or ["*"],
        }

    def project_options_of_interest(self, project):
        """Extract the options concerning this step from the project.

        The prime step doesn't care about any project options.
        """

        return {}
