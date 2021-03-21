# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

from testtools import TestCase
from testtools.matchers import Equals

from snapcraft.plugins.v2.conda import CondaPlugin


class CondaPluginTest(TestCase):
    def test_schema(self):
        schema = CondaPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        "conda-packages": {
                            "type": "array",
                            "minItems": 1,
                            "uniqueItems": True,
                            "items": {"type": "string"},
                            "default": [],
                        },
                        "conda-python-version": {"type": "string", "default": ""},
                        "conda-miniconda-version": {
                            "type": "string",
                            "default": "latest",
                        },
                    },
                }
            ),
        )

    def test_get_build_packages(self):
        plugin = CondaPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_packages(), Equals({"curl"}))

    def test_get_build_environment(self):
        plugin = CondaPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_environment(),
            Equals({"PATH": "${HOME}/miniconda/bin:${PATH}"}),
        )

    def test_get_build_commands(self):
        class Options:
            conda_packages = ["python"]
            conda_python_version = ""
            conda_miniconda_version = "latest"

        plugin = CondaPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    'if ! [ -e "${HOME}/miniconda.sh" ]; then\n    curl --proto \'=https\' --tlsv1.2 -sSf https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh > ${HOME}/miniconda.sh\n    chmod 755 ${HOME}/miniconda.sh\n    export PATH="${HOME}/miniconda/bin:${PATH}"\nfi\n',
                    "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
                    "CONDA_TARGET_PREFIX_OVERRIDE=/snap/${SNAPCRAFT_PROJECT_NAME}/current conda create --prefix $SNAPCRAFT_PART_INSTALL --yes python",
                ]
            ),
        )

    def test_get_build_commands_with_python_version_parameters(self):
        class Options:
            conda_packages = ["python"]
            conda_python_version = "3.7"
            conda_miniconda_version = "latest"

        plugin = CondaPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    'if ! [ -e "${HOME}/miniconda.sh" ]; then\n    curl --proto \'=https\' --tlsv1.2 -sSf https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh > ${HOME}/miniconda.sh\n    chmod 755 ${HOME}/miniconda.sh\n    export PATH="${HOME}/miniconda/bin:${PATH}"\nfi\n',
                    "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
                    "CONDA_TARGET_PREFIX_OVERRIDE=/snap/${SNAPCRAFT_PROJECT_NAME}/current conda create --prefix $SNAPCRAFT_PART_INSTALL --yes python=3.7 python",
                ]
            ),
        )
