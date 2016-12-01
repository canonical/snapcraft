# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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


class State(yaml.YAMLObject):
    def __init__(self, part_properties, project):
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

    def __repr__(self):
        items = sorted(self.__dict__.items())
        strings = (': '.join((key, repr(value))) for key, value in items)
        representation = ', '.join(strings)

        return '{}({})'.format(self.__class__.__name__, representation)

    def __eq__(self, other):
        if type(other) is type(self):
            return self.__dict__ == other.__dict__

        return False
