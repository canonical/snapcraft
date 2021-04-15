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
      List of conda packages to install.

    - conda-python-version
      string
      Python version major and minor version (e.g. 3.8).

    - conda-miniconda-version
      string, default latest
      The version of miniconda to initialize.
"""

import os
import platform
from textwrap import dedent
from typing import Any, Dict, List, Set

from snapcraft.internal.errors import SnapcraftException
from snapcraft.plugins.v2 import PluginV2


_MINICONDA_ARCH_FROM_SNAP_ARCH = {
    "i386": "x86",
    "amd64": "x86_64",
    "armhf": "armv7l",
    "ppc64el": "ppc64le",
}
_MINICONDA_ARCH_FROM_PLATFORM = {"x86_64": {"32bit": "x86", "64bit": "x86_64"}}


class ArchitectureMissing(SnapcraftException):
    def __init__(self, snap_arch: str) -> None:
        self.snap_arch = snap_arch

    def get_brief(self) -> str:
        return (
            f"Architecture {self.snap_arch!r} is not supported with the 'conda' plugin."
        )

    def get_resolution(self) -> str:
        return "Ensure running the build on a supported architecture for this plugin."


def _get_architecture() -> str:
    snap_arch = os.getenv("SNAP_ARCH")
    # The first scenario is the general case as snapcraft will be running from the snap.
    if snap_arch is not None:
        try:
            miniconda_arch = _MINICONDA_ARCH_FROM_SNAP_ARCH[snap_arch]
        except KeyError:
            raise ArchitectureMissing(snap_arch)
    # But there may be times when running from a virtualenv while doing development.
    else:
        miniconda_arch = _MINICONDA_ARCH_FROM_PLATFORM[platform.machine()][
            platform.architecture()[0]
        ]

    return miniconda_arch


def _get_miniconda_source(version: str) -> str:
    """Return tuple of source_url and source_checksum (if known)."""
    arch = _get_architecture()
    source = f"https://repo.anaconda.com/miniconda/Miniconda3-{version}-Linux-{arch}.sh"
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
        conda_target_prefix = "/snap/${SNAPCRAFT_PROJECT_NAME}/current"

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
