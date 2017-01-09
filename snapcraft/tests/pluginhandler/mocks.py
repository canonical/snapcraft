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

import snapcraft
from snapcraft.internal import pluginhandler, project_loader


class TestPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        return {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'type': 'object',
            'additionalProperties': False,
            'properties': {
                'test-property': {
                    'type': 'string'
                }
            },
        }

    @classmethod
    def get_pull_properties(cls):
        return ['test-property']

    @classmethod
    def get_build_properties(cls):
        return ['test-property']


def loadplugin(part_name, plugin_name=None, part_properties=None,
               project_options=None):
    if not plugin_name:
        plugin_name = 'nil'
    properties = {'plugin': plugin_name}
    if part_properties:
        properties.update(part_properties)
    if not project_options:
        project_options = snapcraft.ProjectOptions()

    schema = project_loader.Validator().part_schema
    return pluginhandler.load_plugin(part_name=part_name,
                                     plugin_name=plugin_name,
                                     part_properties=properties,
                                     project_options=project_options,
                                     part_schema=schema)
