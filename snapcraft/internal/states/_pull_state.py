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


def _schema_properties():
    return {
        'plugin',
        'stage-packages',
        'source',
        'source-commit',
        'source-depth',
        'source-tag',
        'source-type',
        'source-branch',
        'source-subdir'
    }


class PullState(State):
    yaml_tag = u'!PullState'

    def __init__(self, property_names, part_properties=None, project=None,
                 stage_packages=None, build_packages=None):
        # Save this off before calling super() since we'll need it
        # FIXME: for 3.x the name `schema_properties` is leaking
        #        implementation details from a higher layer.
        self.schema_properties = property_names
        self.assets = {
            'stage-packages': stage_packages,
            'build-packages': build_packages,
        }

        super().__init__(part_properties, project)

    def properties_of_interest(self, part_properties):
        """Extract the properties concerning this step from part_properties."""

        properties = {}
        for name in self.schema_properties:
            properties[name] = part_properties.get(name)

        for name in _schema_properties():
            properties[name] = part_properties.get(name)

        return properties

    def project_options_of_interest(self, project):
        """Extract the options concerning this step from the project.

        The pull step only cares about the target architecture.
        """

        return {'deb_arch': getattr(project, 'deb_arch', None)}
