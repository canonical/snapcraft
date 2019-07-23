# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

This plugin uses the common plugin keywords as well as those for 'sources'.
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

In the cases where dumping the content needs some mangling or organizing
one would take advantage of the core functionalities available to plugins
such as: `filesets`, `stage`, `snap` and `organize`.
"""

import os
import shutil
import subprocess

import snapcraft
from snapcraft.internal import errors, sources


# Supported versions https://repo.anaconda.com/miniconda/
_MINICONDA_URL = {
    "4.6.14": dict(
        source="https://repo.anaconda.com/miniconda/Miniconda3-4.6.14-Linux-x86_64.sh",
        checksum="md5/718259965f234088d785cad1fbd7de03",
    ),
    "latest": dict(
        source="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh",
        checksum=None,
    ),
}

_MINICONDA_UNKNOWN_VERSION_URL = dict(
    source="https://repo.anaconda.com/miniconda/Miniconda3-{}-Linux-x86_64.sh",
    checksum=None,
)


class MinicondaBadArchitectureError(snapcraft.internal.errors.SnapcraftError):

    fmt = (
        "Failed to fetch miniconda: "
        "The architecture {architecture!r} is not supported."
    )

    def __init__(self, *, architecture: str) -> None:
        super().__init__(architecture=architecture)


class CondaPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["conda-packages"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["conda-python-version"] = {"type": "string", "default": ""}
        schema["properties"]["conda-miniconda-version"] = {
            "type": "string",
            "default": "latest",
        }
        schema["required"] = ["conda-packages"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ["conda-miniconda-version"]

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ["conda-python-version", "conda-packages"]

    def __init__(self, name, options, project) -> None:
        super().__init__(name, options, project)
        if project.info.get_build_base() not in ("core", "core16", "core18"):
            raise errors.PluginBaseError(
                part_name=self.name, base=project.info.get_build_base()
            )

        self._conda_home = os.path.join(self.partdir, "miniconda")
        self._miniconda_script = os.path.join(self.partdir, "miniconda.sh")

    def _get_miniconda_source(self) -> sources.Script:
        miniconda_url = _MINICONDA_URL.get(self.options.conda_miniconda_version)
        if not miniconda_url:
            miniconda_url = _MINICONDA_UNKNOWN_VERSION_URL
            miniconda_url["source"] = miniconda_url["source"].format(
                self.options.conda_miniconda_version
            )

        return sources.Script(
            source=miniconda_url["source"],
            source_dir=self.partdir,
            source_checksum=miniconda_url["checksum"],
        )

    def pull(self):
        if self.project.deb_arch != "amd64":
            raise MinicondaBadArchitectureError(architecture=self.project.deb_arch)
        miniconda_source_script = self._get_miniconda_source()
        miniconda_source_script.download(filepath=self._miniconda_script)
        os.chmod(self._miniconda_script, mode=0o755)

    def clean_pull(self):
        super().clean_pull()

        # Remove conda_home (if present)
        if os.path.exists(self._conda_home):
            shutil.rmtree(self._conda_home)

    def build(self):
        super().build()

        conda_cmd = os.path.join(self._conda_home, "bin", "conda")

        # -b: batch mode (agrees to license and does not inject into bash profile)
        # -f: force
        # -p: prefix
        if not os.path.exists(conda_cmd):
            subprocess.check_call([self._miniconda_script, "-bfp", self._conda_home])

        deploy_cmd = [conda_cmd, "create", "--prefix", self.installdir, "--yes"]
        if self.options.conda_python_version:
            deploy_cmd.append("python={}".format(self.options.conda_python_version))

        # conda needs to rewrite the prefixes in the python shebangs and binaries
        conda_target_prefix = os.path.join(
            os.path.sep, "snap", self.project.info.name, "current"
        )

        subprocess.check_call(
            deploy_cmd + self.options.conda_packages,
            env=dict(CONDA_TARGET_PREFIX_OVERRIDE=conda_target_prefix),
        )
