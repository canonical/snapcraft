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

"""The make plugin is useful for building make based parts.

Make based projects are projects that have a Makefile that drives the
build.

This plugin always runs 'make' followed by 'make install', except when
the 'artifacts' keyword is used.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - artifacts:
      (list)
      Link/copy the given files from the make output to the snap
      installation directory. If specified, the 'make install'
      step will be skipped.

    - makefile:
      (string)
      Use the given file as the makefile.

    - make-parameters:
      (list of strings)
      Pass the given parameters to the make command.

    - make-install-var:
      (string; default: DESTDIR)
      Use this variable to redirect the installation into the snap.
"""

import os
import snapcraft
import snapcraft.common


class MakePlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["makefile"] = {"type": "string"}
        schema["properties"]["make-parameters"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["make-install-var"] = {
            "type": "string",
            "default": "DESTDIR",
        }
        schema["properties"]["artifacts"] = {
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
        return ["makefile", "make-parameters", "make-install-var", "artifacts"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append("make")

    def make(self, env=None):
        command = ["make"]

        if self.options.makefile:
            command.extend(["-f", self.options.makefile])

        if self.options.make_parameters:
            command.extend(self.options.make_parameters)

        self.run(command + ["-j{}".format(self.parallel_build_count)], env=env)
        if self.options.artifacts:
            for artifact in self.options.artifacts:
                source_path = os.path.join(self.builddir, artifact)
                destination_path = os.path.join(self.installdir, artifact)
                if os.path.isdir(source_path):
                    snapcraft.file_utils.link_or_copy_tree(
                        source_path, destination_path
                    )
                else:
                    snapcraft.file_utils.link_or_copy(source_path, destination_path)
        else:
            command.append("install")
            if self.options.make_install_var:
                command.append(
                    "{}={}".format(self.options.make_install_var, self.installdir)
                )

            self.run(command, env=env)

    def build(self):
        super().build()
        self.make()
