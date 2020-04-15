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

from testtools.matchers import Equals
from testtools import TestCase

from snapcraft.plugins.v2.python import PythonPlugin


class PythonPluginTest(TestCase):
    def test_schema(self):
        schema = PythonPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        "constraints": {
                            "default": [],
                            "items": {"type": "string"},
                            "minitems": 1,
                            "type": "array",
                            "uniqueItems": True,
                        },
                        "python-packages": {
                            "default": [],
                            "items": {"type": "string"},
                            "minitems": 1,
                            "type": "array",
                            "uniqueItems": True,
                        },
                        "requirements": {
                            "default": [],
                            "items": {"type": "string"},
                            "minitems": 1,
                            "type": "array",
                            "uniqueItems": True,
                        },
                    },
                    "type": "object",
                }
            ),
        )

    def test_get_build_packages(self):
        plugin = PythonPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_packages(),
            Equals({"findutils", "python3-venv", "python3-dev"}),
        )

    def test_get_build_environment(self):
        plugin = PythonPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_environment(),
            Equals(
                {
                    "SNAPCRAFT_PYTHON_HOST_INTERPRETER": "/usr/bin/python3",
                    "SNAPCRAFT_PYTHON_VENV_ARGS": "",
                }
            ),
        )

    def test_get_build_commands(self):
        class Options:
            constraints = list()
            requirements = list()
            python_packages = list()

        plugin = PythonPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    '"${SNAPCRAFT_PYTHON_HOST_INTERPRETER}" -m venv ${SNAPCRAFT_PYTHON_VENV_ARGS} '
                    '"${SNAPCRAFT_PART_INSTALL}"',
                    '. "${SNAPCRAFT_PART_INSTALL}/bin/activate"',
                    "[ -f setup.py ] && pip install .",
                    'for e in $(find "${SNAPCRAFT_PART_INSTALL}" -type f -executable); do if head '
                    '-1 "${e}" | grep -q "python" ; then sed -r "1 '
                    's|#\\!.*python3?$|#\\!/usr/bin/env python|" -i "${e}"; fi ; done',
                ]
            ),
        )

    def test_get_build_commands_with_all_properties(self):
        class Options:
            constraints = ["constraints.txt"]
            requirements = ["requirements.txt"]
            python_packages = ["pip"]

        plugin = PythonPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    '"${SNAPCRAFT_PYTHON_HOST_INTERPRETER}" -m venv ${SNAPCRAFT_PYTHON_VENV_ARGS} '
                    '"${SNAPCRAFT_PART_INSTALL}"',
                    '. "${SNAPCRAFT_PART_INSTALL}/bin/activate"',
                    "pip install -c 'constraints.txt' -U pip",
                    "pip install -c 'constraints.txt' -U -r 'requirements.txt'",
                    "[ -f setup.py ] && pip install .",
                    'for e in $(find "${SNAPCRAFT_PART_INSTALL}" -type f -executable); do if head '
                    '-1 "${e}" | grep -q "python" ; then sed -r "1 '
                    's|#\\!.*python3?$|#\\!/usr/bin/env python|" -i "${e}"; fi ; done',
                ]
            ),
        )
