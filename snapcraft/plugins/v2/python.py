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

"""The python plugin can be used for python 2 or 3 based parts.

It can be used for python projects where you would want to do:

    - import python modules with a requirements.txt
    - build a python project that has a setup.py
    - install packages straight from pip

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - requirements:
      (list of strings)
      List of paths to requirements files.
    - constraints:
      (list of strings)
      List of paths to constraint files.
    - python-packages:
      (list)
      A list of dependencies to get from PyPI. If needed, pip,
      setuptools and wheel can be upgraded here.

If the plugin finds a python interpreter with a basename that matches
`python-version` in the <stage> directory on the following fixed path:
`<stage-dir>/usr/bin/<python-interpreter>` then this interpreter would
be preferred instead and no interpreter would be brought in through
`stage-packages` mechanisms.
"""

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
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "constraints": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "python-packages": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
        }

    def get_build_packages(self) -> Set[str]:
        return {"findutils", "python3-dev", "python3-venv"}

    def get_build_environment(self) -> Dict[str, str]:
        return {
            "SNAPCRAFT_PYTHON_HOST_INTERPRETER": "/usr/bin/python3",
            "SNAPCRAFT_PYTHON_VENV_ARGS": "",
        }

    def get_build_commands(self) -> List[str]:
        build_commands = [
            '"${SNAPCRAFT_PYTHON_HOST_INTERPRETER}" -m venv ${SNAPCRAFT_PYTHON_VENV_ARGS} "${SNAPCRAFT_PART_INSTALL}"',
            '. "${SNAPCRAFT_PART_INSTALL}/bin/activate"',
        ]

        if self.options.constraints:
            constraints = " ".join(f"-c {c!r}" for c in self.options.constraints)
        else:
            constraints = ""

        if self.options.python_packages:
            python_packages = " ".join(self.options.python_packages)
            python_packages_cmd = f"pip install {constraints} -U {python_packages}"
            build_commands.append(python_packages_cmd)

        if self.options.requirements:
            requirements = " ".join(f"-r {r!r}" for r in self.options.requirements)
            requirements_cmd = f"pip install {constraints} -U {requirements}"
            build_commands.append(requirements_cmd)

        build_commands.append("[ -f setup.py ] && pip install .")

        # Now fix shebangs.
        # TODO: replace with snapcraftctl once the two scripts are consolidated
        # and use mangling.rewrite_python_shebangs.
        build_commands.append(
            'for e in $(find "${SNAPCRAFT_PART_INSTALL}" -type f -executable); do '
            'if head -1 "${e}" | grep -q "python" ; then '
            'sed -r "1 s|#\\!.*python3?$|#\\!/usr/bin/env python|" -i "${e}"; '
            "fi ; "
            "done"
        )

        return build_commands
