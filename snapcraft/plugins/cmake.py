# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

"""The cmake plugin is useful for building cmake based parts.

These are projects that have a CMakeLists.txt that drives the build.
The plugin requires a CMakeLists.txt in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - configflags:
      (list of strings)
      configure flags to pass to the build using the common cmake semantics.
"""

import snapcraft.plugins.make
from ._cmake import cmakepluginbase


class CMakePlugin(cmakepluginbase.CMakePluginBase):

    @classmethod
    def schema(cls):
        schema = super().schema()
        makeprops = snapcraft.plugins.make.MakePlugin.schema()['properties']
        schema['properties'].update(makeprops)
        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return (super().get_build_properties() +
                snapcraft.plugins.make.MakePlugin.get_build_properties())

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.make = snapcraft.plugins.make.MakePlugin(name, options, project)

    def runCompilation(self):
        self.make.make(env=self._build_environment())
