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

"""The WAF plugin is useful to build waf based parts

waf bases projects are projects that drive configuration and build via
a local waf python helper - see https://github.com/waf-project/waf for more
details.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

In addition, this plugin uses the following plugin-specific keywords:

    - configflags:
      (list of strings)
      configure flags to pass to the build such as those shown by running
      ./waf --help
"""

import snapcraft


class WafPlugin(snapcraft.BasePlugin):
    """plugin to build via waf build system"""

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["configflags"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(["python-dev:native"])

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["configflags"]

    def env(self, root):
        env = super().env(root)
        if self.project.is_cross_compiling:
            env.extend(
                [
                    "CC={}-gcc".format(self.project.arch_triplet),
                    "CXX={}-g++".format(self.project.arch_triplet),
                ]
            )
        return env

    def enable_cross_compilation(self):
        # Let snapcraft know that this plugin can cross-compile
        # If the method isn't implemented an exception is raised
        pass

    def build(self):
        super().build()
        self.run(["./waf", "distclean"])
        self.run(["./waf", "configure"] + self.options.configflags)
        self.run(["./waf", "build"])
        self.run(
            ["./waf", "install", "--destdir=" + self.installdir]
        )  # target from snappy env
