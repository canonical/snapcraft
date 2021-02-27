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

"""Create snaps from conda packages.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - conda-packages
      (list of strings, default [])
      conda packages to install

    - conda-python-version
      string
      python version like 3.8, etc.

    - conda-miniconda-version
      string, default latest
      the miniconda to initialize.
"""

import os
from textwrap import dedent
from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2


def _get_miniconda_source(version: str) -> str:
    """Return tuuple of source_url and source_checksum (if known)."""
    source = f"https://repo.anaconda.com/miniconda/Miniconda3-{version}-Linux-x86_64.sh"
    return source


class CondaPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
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
                "conda-miniconda-version": {"type": "string", "default": "latest"},
            },
        }

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {"curl"}

    def get_build_environment(self) -> Dict[str, str]:
        return {"PATH": "${HOME}/miniconda/bin:${PATH}"}

    def _get_download_miniconda_command(self, url: str) -> str:
        return dedent(
            f"""\
        if ! [ -e "${{HOME}}/miniconda.sh" ]; then
            curl --proto '=https' --tlsv1.2 -sSf {url} > ${{HOME}}/miniconda.sh
            chmod 755 ${{HOME}}/miniconda.sh
            export PATH="${{HOME}}/miniconda/bin:${{PATH}}"
        fi
        """
        )

    def _get_install_env_command(self) -> str:
        cmd = ["${HOME}/miniconda.sh", "-bfp", "${HOME}/miniconda"]
        return " ".join(cmd)

    def _get_deploy_command(self) -> str:
        import pprint as pp

        pp.pprint(self.options)
        conda_target_prefix = os.path.join(
            # os.path.sep, "snap", self.project._snap_meta.name, "current"
            os.path.sep,
            "snap",
            "tfengine",
            "current",
        )

        deploy_cmd = [
            "CONDA_TARGET_PREFIX_OVERRIDE=" + conda_target_prefix,
            "conda",
            "create",
            "--prefix",
            "$SNAPCRAFT_PART_INSTALL",
            "--yes",
        ]
        if self.options.conda_python_version:
            deploy_cmd.append("python={}".format(self.options.conda_python_version))

        deploy_cmd.extend(self.options.conda_packages)

        return " ".join(deploy_cmd)

    def get_build_commands(self) -> List[str]:
        url = _get_miniconda_source(self.options.conda_miniconda_version)
        return [
            self._get_download_miniconda_command(url),
            self._get_install_env_command(),
            self._get_deploy_command(),
        ]
