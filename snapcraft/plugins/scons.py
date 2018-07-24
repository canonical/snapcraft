# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

"""The scons plugin is useful for building parts that build with scons.

These are projects that have a SConstruct that drives the build.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - scons-options:
      (list of strings)
      flags to pass to the build using the scons semantics for parameters.
"""

import os

import snapcraft


class SconsPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["scons-options"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["scons-options"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append("scons")

    def build(self):
        super().build()
        env = os.environ.copy()
        env["DESTDIR"] = self.installdir
        self.run(["scons"] + self.options.scons_options)
        self.run(["scons", "install"] + self.options.scons_options, env=env)
