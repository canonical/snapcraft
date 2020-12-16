# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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
from typing import Optional, Tuple

from snapcraft.internal import errors, sources
from snapcraft.plugins.v1 import PluginV1

_MINICONDA_CHECKSUMS = {"4.6.14": "md5/718259965f234088d785cad1fbd7de03"}


def _get_miniconda_source(version: str) -> Tuple[str, Optional[str]]:
    """Return tuuple of source_url and source_checksum (if known)."""
    source = f"https://repo.anaconda.com/miniconda/Miniconda3-{version}-Linux-x86_64.sh"
    checksum: Optional[str] = _MINICONDA_CHECKSUMS.get(version, None)
    return source, checksum


class MinicondaBadArchitectureError(errors.SnapcraftError):

    fmt = (
        "Failed to fetch miniconda: "
        "The architecture {architecture!r} is not supported."
    )

    def __init__(self, *, architecture: str) -> None:
        super().__init__(architecture=architecture)


class CondaPlugin(PluginV1):
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
        self._conda_home = os.path.join(self.partdir, "miniconda")
        self._miniconda_script = os.path.join(self.partdir, "miniconda.sh")

    def _get_miniconda_script(self) -> sources.Script:
        source, checksum = _get_miniconda_source(self.options.conda_miniconda_version)

        return sources.Script(
            source=source, source_dir=self.partdir, source_checksum=checksum
        )

    def pull(self):
        if self.project.deb_arch != "amd64":
            raise MinicondaBadArchitectureError(architecture=self.project.deb_arch)
        miniconda_source_script = self._get_miniconda_script()
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
            os.path.sep, "snap", self.project._snap_meta.name, "current"
        )

        subprocess.check_call(
            deploy_cmd + self.options.conda_packages,
            env=dict(CONDA_TARGET_PREFIX_OVERRIDE=conda_target_prefix),
        )
