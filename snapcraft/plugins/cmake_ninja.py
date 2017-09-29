# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

"""The cmakeninja plugin is useful for building cmake based parts using ninja.

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

import snapcraft.plugins.ninja
from ._cmake import cmakepluginbase


class CMakeNinjaPlugin(cmakepluginbase.CMakePluginBase):

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.ninja = snapcraft.plugins.ninja.NinjaPlugin(name, options,
                                                         project)
        self.build_packages += self.ninja.build_packages

    def configflags(self):
        configs = super().configflags()
        configs.append('-GNinja')
        return configs

    def runCompilation(self):
        env = self._build_environment()
        env['DESTDIR'] = self.installdir
        return self.ninja.ninja('install', env)
