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

"""The python plugin can be used for 3 based parts.

It can be used for python projects where you would want to do:

    - import python modules with a requirements.txt
    - build a python project that has a setup.py
    - install packages straight from pip

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - requirements
      (list of strings)
      List of paths to requirements files.

    - constraints
      (list of strings)
      List of paths to constraint files.

    - python-packages
      (list)
      A list of dependencies to get from PyPI. If needed, pip,
      setuptools and wheel can be upgraded here.

This plugin also interprets these specific build-environment entries:

    - SNAPCRAFT_PYTHON_INTERPRETER
      (default: python3)
      The interpreter binary to search for in PATH.

    - SNAPCRAFT_PYTHON_VENV_ARGS
      Additional arguments for venv.

By default this plugin uses python from the base, if a part using
this plugin uses a build-base other than that of the base, or a
different interpreter is desired, it must be bundled in the snap
(including venv) and must be in PATH.

It is required to bundle python when creating a snap that uses
classic confinement, this can be accomplished on Ubuntu by
adding stage-packages (i.e.; python3-venv).

Use of python3-<python-package> in stage-packages will force the
inclusion of the python interpreter.
"""

import shlex
from textwrap import dedent
from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2


class PythonPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "requirements": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "constraints": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "python-packages": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
        }

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {"findutils", "python3-dev", "python3-venv"}

    def get_build_environment(self) -> Dict[str, str]:
        return {
            # Add PATH to the python interpreter we always intend to use with
            # this plugin. It can be user overridden, but that is an explicit
            # choice made by a user.
            "PATH": "${SNAPCRAFT_PART_INSTALL}/bin:${PATH}",
            "SNAPCRAFT_PYTHON_INTERPRETER": "python3",
            "SNAPCRAFT_PYTHON_VENV_ARGS": "",
        }

    def get_build_commands(self) -> List[str]:
        build_commands = [
            '"${SNAPCRAFT_PYTHON_INTERPRETER}" -m venv ${SNAPCRAFT_PYTHON_VENV_ARGS} "${SNAPCRAFT_PART_INSTALL}"'
        ]

        if self.options.constraints:
            constraints = " ".join(f"-c {c!r}" for c in self.options.constraints)
        else:
            constraints = ""

        if self.options.python_packages:
            python_packages = " ".join(
                [shlex.quote(pkg) for pkg in self.options.python_packages]
            )
            python_packages_cmd = f"pip install {constraints} -U {python_packages}"
            build_commands.append(python_packages_cmd)

        if self.options.requirements:
            requirements = " ".join(f"-r {r!r}" for r in self.options.requirements)
            requirements_cmd = f"pip install {constraints} -U {requirements}"
            build_commands.append(requirements_cmd)

        build_commands.append(f"[ -f setup.py ] && pip install {constraints} -U .")

        # Now fix shebangs.
        # TODO: replace with snapcraftctl once the two scripts are consolidated
        # and use mangling.rewrite_python_shebangs.
        build_commands.append(
            dedent(
                """\
            for e in $(find "${SNAPCRAFT_PART_INSTALL}" -type f -executable)
            do
                if head -1 "${e}" | grep -q "python" ; then
                    sed \\
                        -r '1 s|#\\!.*python3?$|#\\!/usr/bin/env '${SNAPCRAFT_PYTHON_INTERPRETER}'|' \\
                        -i "${e}"
                fi
            done
        """
            )
        )

        # Lastly, fix the symlink to the "real" python3 interpreter.
        # TODO: replace with snapcraftctl (create_relative_symlinks).
        build_commands.append(
            dedent(
                """\
            interp_path="${SNAPCRAFT_PART_INSTALL}/bin/${SNAPCRAFT_PYTHON_INTERPRETER}"
            if [ -f "${interp_path}" ]; then
                current_link=$(readlink "${interp_path}")
                # Change link if in $SNAPCRAFT_PART_INSTALL
                if echo "${current_link}" | grep -q "${SNAPCRAFT_PART_INSTALL}" ; then
                    new_link=$(realpath \\
                               --strip \\
                               --relative-to="${SNAPCRAFT_PART_INSTALL}/bin/" \\
                              "${current_link}")
                    rm "${interp_path}"
                    ln -s "${new_link}" "${interp_path}"
                fi
            fi
        """
            )
        )

        return build_commands
