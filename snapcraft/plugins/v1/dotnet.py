# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017, 2020 Canonical Ltd
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

"""The dotnet plugin is used to build dotnet core runtime parts.'

The plugin uses the dotnet SDK to install dependencies from nuget
and follows standard semantics from a dotnet core project.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

The plugin will take into account the following build-attributes:

    - debug: builds using a Debug configuration.

"""

import fnmatch
import json
import os
import shutil
import urllib.parse
import urllib.request
from typing import List

from snapcraft import formatting_utils, sources
from snapcraft.internal import errors
from snapcraft.plugins.v1 import PluginV1

_DOTNET_RELEASE_METADATA_URL = "https://dotnetcli.blob.core.windows.net/dotnet/release-metadata/{version}/releases.json"  # noqa
_RUNTIME_DEFAULT = "2.0.9"
_VERSION_DEFAULT = "2.0"

# TODO extend for other architectures
_SDK_ARCH = ["amd64"]


class DotNetBadArchitectureError(errors.SnapcraftError):

    fmt = (
        "Failed to prepare the .NET SDK: "
        "The architecture {architecture!r} is not supported. "
        "Supported architectures are: {supported}."
    )

    def __init__(self, *, architecture: str, supported: List[str]) -> None:
        super().__init__(
            architecture=architecture,
            supported=formatting_utils.humanize_list(supported, "and"),
        )


class DotNetBadReleaseDataError(errors.SnapcraftError):

    fmt = (
        "Failed to prepare the .NET SDK: "
        "An error occurred while fetching the version details "
        "for {version!r}. Check that the version is correct."
    )

    def __init__(self, version):
        super().__init__(version=version)


class DotNetPlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["dotnet-version"] = {
            "type": "number",
            "default": _VERSION_DEFAULT,
        }
        schema["properties"]["dotnet-runtime-version"] = {
            "type": "string",
            "default": _RUNTIME_DEFAULT,
        }
        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["dotnet-runtime-version", "dotnet-version"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._dotnet_dir = os.path.join(self.partdir, "dotnet")
        self._dotnet_sdk_dir = os.path.join(self._dotnet_dir, "sdk")

        self._setup_base_tools(project._get_build_base())

        self._sdk = self._get_sdk()
        self._dotnet_cmd = os.path.join(self._dotnet_sdk_dir, "dotnet")

    def _setup_base_tools(self, base):
        extra_packages = [
            "libcurl3",
            "libcurl3-gnutls",
            "liblttng-ust0",
            "libunwind8",
            "lldb",
            "libssl1.0.0",
            "libgssapi-krb5-2",
            "zlib1g",
            "libgcc1",
        ]
        if base in ("core", "core16"):
            self.stage_packages += extra_packages + ["libicu55"]
        elif base in ("core18",):
            self.stage_packages += extra_packages + ["libicu60"]
        else:
            raise errors.PluginBaseError(part_name=self.name, base=base)

    def _get_sdk(self):
        sdk_arch = self.project.deb_arch
        if sdk_arch not in _SDK_ARCH:
            raise DotNetBadArchitectureError(architecture=sdk_arch, supported=_SDK_ARCH)
        # TODO: Make this a class that takes care of retrieving the infos
        sdk_files = self._get_sdk_info(self.options.dotnet_runtime_version)

        return sources.Tar(sdk_files["url"], self._dotnet_sdk_dir)

    def pull(self):
        super().pull()

        os.makedirs(self._dotnet_sdk_dir, exist_ok=True)

        self._sdk.pull()

    def clean_pull(self):
        super().clean_pull()

        # Remove the dotnet directory (if any)
        if os.path.exists(self._dotnet_dir):
            shutil.rmtree(self._dotnet_dir)

    def build(self):
        super().build()

        if "debug" in self.options.build_attributes:
            configuration = "Debug"
        else:
            configuration = "Release"

        self.run([self._dotnet_cmd, "build", "-c", configuration])

        publish_cmd = [
            self._dotnet_cmd,
            "publish",
            "-c",
            configuration,
            "-o",
            self.installdir,
        ]
        # Build command for self-contained application
        publish_cmd += ["--self-contained", "-r", "linux-x64"]
        self.run(publish_cmd)

        # Workaround to set the right permission for the executable.
        appname = os.path.join(self.installdir, self._get_appname())
        if os.path.exists(appname):
            os.chmod(appname, 0o755)

    def _get_appname(self):
        for file in os.listdir(self.builddir):
            if fnmatch.fnmatch(file, "*.??proj"):
                return os.path.splitext(file)[0]

    def _get_version_metadata(self, version):
        json_data = self._get_dotnet_release_metadata()
        package_data = list(
            filter(lambda x: x.get("release-version") == version, json_data["releases"])
        )

        if not package_data:
            raise DotNetBadReleaseDataError(version)

        return package_data[0]

    def _get_dotnet_release_metadata(self):
        package_metadata = []

        metadata_url = _DOTNET_RELEASE_METADATA_URL.format(
            version=self.options.dotnet_version
        )
        req = urllib.request.Request(metadata_url)
        r = urllib.request.urlopen(req).read()
        package_metadata = json.loads(r.decode("utf-8"))

        return package_metadata

    def _get_sdk_info(self, version):
        metadata = self._get_version_metadata(version)

        sdk_files = list(
            filter(lambda x: x.get("rid") == "linux-x64", metadata["sdk"]["files"])
        )

        if not sdk_files:
            raise DotNetBadReleaseDataError(version)

        return sdk_files[0]

    def env(self, root):
        # Update the PATH only during the Build and Install step
        if root == self.installdir:
            return ["PATH={}:$PATH".format(self._dotnet_sdk_dir)]
        else:
            return []
