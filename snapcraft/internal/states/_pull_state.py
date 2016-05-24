# -*- Mode:Python; indent-tabs-buildnil; tab-width:4 -*-
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

from snapcraft.internal.states._state import State


def _pull_state_constructor(loader, node):
    parameters = loader.construct_mapping(node)
    return PullState(**parameters)

yaml.add_constructor(u'!PullState', _pull_state_constructor)


class PullState(State):
    yaml_tag = u'!PullState'

    def __init__(self, schema_properties, options=None, project=None):
        # Save this off before calling super() since we'll need it
        self.schema_properties = schema_properties

        super().__init__(options, project)

    def properties_of_interest(self, options):
        """Extract the properties concerning this step from the options.

        The properties of interest to the pull step vary depending on the
        plugin, so we use self.schema_properties.
        """

        properties = {}
        for schema_property in self.schema_properties:
            properties[schema_property] = getattr(options, schema_property,
                                                  None)

        return properties

    def project_options_of_interest(self, project):
        """Extract the options concerning this step from the project.

        The pull step only cares about the target architecture.
        """

        return {'deb_arch': getattr(project, 'deb_arch', None)}
