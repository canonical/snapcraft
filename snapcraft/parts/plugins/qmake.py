# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

"""The qmake plugin is useful for building qmake-based parts.

These are projects that are built using .pro files.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - qmake-parameters:
      (list of strings)
      additional options to pass to the qmake invocation.

    - qmake-project-file:
      (string)
      the qmake project file to use. This is usually only needed if
      qmake can not determine what project file to use on its own.
"""

from typing import Any, Dict, List, Set

from craft_parts import plugins


class QMakePlugin(plugins.Plugin):

    properties_class = QMakePluginProperties

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {"g++", "make", "qt5-qmake"}

    def get_build_environment(self) -> Dict[str, str]:
        return {"QT_SELECT": "qt5"}

    @property
    def out_of_source_build(self):
        return True

    def _get_qmake_configure_command(self) -> str:
        cmd = [
            "qmake",
            'QMAKE_CFLAGS+="${CFLAGS:-}"',
            'QMAKE_CXXFLAGS+="${CXXFLAGS:-}"',
            'QMAKE_LFLAGS+="${LDFLAGS:-}"',
        ] + self.options.qmake_parameters

        if self.options.qmake_project_file:
            cmd.append(
                '"${{CRAFT_PART_SRC_WORK}}/{}"'.format(
                    self.options.qmake_project_file
                )
            )
        else:
            cmd.append('"${CRAFT_PART_SRC_WORK}"')

        return " ".join(cmd)

    def get_build_commands(self) -> List[str]:
        return [
            self._get_qmake_configure_command(),
            # Avoid overriding the CFLAGS and CXXFLAGS environment
            # variables qmake sets in the generated Makefile
            f'env -u CFLAGS -u CXXFLAGS make "-j{self._part_info.parallel_build_count}"',
            f'make install INSTALL_ROOT="{self._part_info.part_install_dir}"',
        ]
