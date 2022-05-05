# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""The conda plugin."""

import os
import platform
import textwrap
from typing import Any, Dict, List, Optional, Set, cast

from craft_parts import plugins
from overrides import overrides

from snapcraft import errors

_MINICONDA_ARCH_FROM_SNAP_ARCH = {
    "i386": "x86",
    "amd64": "x86_64",
    "armhf": "armv7l",
    "ppc64el": "ppc64le",
}
_MINICONDA_ARCH_FROM_PLATFORM = {"x86_64": {"32bit": "x86", "64bit": "x86_64"}}


def _get_architecture() -> str:
    snap_arch = os.getenv("SNAP_ARCH")
    # The first scenario is the general case as snapcraft will be running from the snap.
    if snap_arch is not None:
        try:
            miniconda_arch = _MINICONDA_ARCH_FROM_SNAP_ARCH[snap_arch]
        except KeyError as key_error:
            raise errors.SnapcraftError(
                f"Architecture not supported for conda plugin: {snap_arch!r}"
            ) from key_error
    # But there may be times when running from a virtualenv while doing development.
    else:
        machine = platform.machine()
        architecture = platform.architecture()[0]
        miniconda_arch = _MINICONDA_ARCH_FROM_PLATFORM[machine][architecture]

    return miniconda_arch


def _get_miniconda_source(version: str) -> str:
    """Return tuple of source_url and source_checksum (if known)."""
    arch = _get_architecture()
    source = f"https://repo.anaconda.com/miniconda/Miniconda3-{version}-Linux-{arch}.sh"
    return source


class CondaPluginProperties(plugins.PluginProperties, plugins.PluginModel):
    """The part properties used by the conda plugin."""

    # part properties required by the plugin
    conda_packages: Optional[List[str]] = None
    conda_python_version: Optional[str] = None
    conda_miniconda_version: str = "latest"

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "CondaPluginProperties":
        """Populate class attributes from the part specification.

        :param data: A dictionary containing part properties.

        :return: The populated plugin properties data object.

        :raise pydantic.ValidationError: If validation fails.
        """
        plugin_data = plugins.extract_plugin_properties(
            data,
            plugin_name="conda",
        )
        return cls(**plugin_data)


class CondaPlugin(plugins.Plugin):
    """A plugin for conda projects.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:
        - conda-packages
          (list of packages, default: None)
          List of packages for conda to install.
        - conda-python-version
          (str, default: None)
          Python version for conda to use (i.e. "3.9").
        - conda-miniconda-version
          (str, default: latest)
          The version of miniconda to initialize.
    """

    properties_class = CondaPluginProperties

    @overrides
    def get_build_snaps(self) -> Set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> Set[str]:
        return set()

    @overrides
    def get_build_environment(self) -> Dict[str, str]:
        return {"PATH": "${HOME}/miniconda/bin:${PATH}"}

    @staticmethod
    def _get_download_miniconda_command(url: str) -> str:
        return textwrap.dedent(
            f"""\
        if ! [ -e "${{HOME}}/miniconda.sh" ]; then
            curl --proto '=https' --tlsv1.2 -sSf {url} > ${{HOME}}/miniconda.sh
            chmod 755 ${{HOME}}/miniconda.sh
        fi"""
        )

    def _get_deploy_command(self, options) -> str:
        conda_target_prefix = f"/snap/{self._part_info.project_name}/current"

        deploy_cmd = [
            f"CONDA_TARGET_PREFIX_OVERRIDE={conda_target_prefix}",
            "conda",
            "create",
            "--prefix",
            str(self._part_info.part_install_dir),
            "--yes",
        ]
        if options.conda_python_version:
            deploy_cmd.append(f"python={options.conda_python_version}")

        if options.conda_packages:
            deploy_cmd.extend(options.conda_packages)

        return " ".join(deploy_cmd)

    @overrides
    def get_build_commands(self) -> List[str]:
        options = cast(CondaPluginProperties, self._options)
        url = _get_miniconda_source(options.conda_miniconda_version)
        return [
            self._get_download_miniconda_command(url),
            "${HOME}/miniconda.sh -bfp ${HOME}/miniconda",
            self._get_deploy_command(options),
        ]
