# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import os
import shutil
import fnmatch
import urllib.request
import json

import snapcraft
from snapcraft import sources
from snapcraft import formatting_utils
from typing import List


_DOTNET_RELEASE_METADATA_URL = (
    "https://raw.githubusercontent.com/dotnet/core/master/release-notes/releases.json"
)  # noqa
_RUNTIME_DEFAULT = "2.0.5"

# TODO extend for other architectures
_SDK_ARCH = ["amd64"]


class DotNetBadArchitectureError(snapcraft.internal.errors.SnapcraftError):

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


class DotNetBadReleaseDataError(snapcraft.internal.errors.SnapcraftError):

    fmt = (
        "Failed to prepare the .NET SDK: "
        "An error occurred while fetching the version details "
        "for {version!r}. Check that the version is correct."
    )

    def __init__(self, version):
        super().__init__(version=version)


class DotNetPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["dotnet-runtime-version"] = {
            "type": "string",
            "default": _RUNTIME_DEFAULT,
        }

        if "required" in schema:
            del schema["required"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["dotnet-runtime-version"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._dotnet_dir = os.path.join(self.partdir, "dotnet")
        self._dotnet_sdk_dir = os.path.join(self._dotnet_dir, "sdk")

        self.stage_packages.extend(
            [
                "libcurl3",
                "libcurl3-gnutls",
                "libicu55",
                "liblttng-ust0",
                "libunwind8",
                "lldb",
                "libssl1.0.0",
                "libgssapi-krb5-2",
                "libc6",
                "zlib1g",
                "libgcc1",
            ]
        )

        self._sdk = self._get_sdk()
        self._dotnet_cmd = os.path.join(self._dotnet_sdk_dir, "dotnet")

    def _get_sdk(self):
        sdk_arch = self.project.deb_arch
        if sdk_arch not in _SDK_ARCH:
            raise DotNetBadArchitectureError(architecture=sdk_arch, supported=_SDK_ARCH)
        # TODO: Make this a class that takes care of retrieving the infos
        sdk_info = self._get_sdk_info(self.options.dotnet_runtime_version)

        sdk_url = sdk_info["package_url"]
        return sources.Tar(
            sdk_url, self._dotnet_sdk_dir, source_checksum=sdk_info["checksum"]
        )

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
        jsonData = self._get_dotnet_release_metadata()
        package_data = list(
            filter(lambda x: x.get("version-runtime") == version, jsonData)
        )

        if not package_data:
            raise DotNetBadReleaseDataError(version)

        return package_data

    def _get_dotnet_release_metadata(self):
        package_metadata = []

        req = urllib.request.Request(_DOTNET_RELEASE_METADATA_URL)
        r = urllib.request.urlopen(req).read()
        package_metadata = json.loads(r.decode("utf-8"))

        return package_metadata

    def _get_sdk_info(self, version):
        metadata = self._get_version_metadata(version)

        if "sdk-linux-x64" in metadata[0]:
            # look for sdk-linux-x64 property, if it doesn't exist
            # look for ubuntu.14.04 entry as shipped during 1.1
            sdk_packge_name = metadata[0]["sdk-linux-x64"]
        elif "sdk-ubuntu.14.04" in metadata[0]:
            sdk_packge_name = metadata[0]["sdk-ubuntu.14.04"]
        else:
            raise DotNetBadReleaseDataError(version)

        sdk_package_url = "{}{}".format(metadata[0]["blob-sdk"], sdk_packge_name)
        sdk_checksum = self._get_package_checksum(
            metadata[0]["checksums-sdk"], sdk_packge_name, version
        )

        return {"package_url": sdk_package_url, "checksum": sdk_checksum}

    def _get_package_checksum(
        self, checksum_url: str, filename: str, version: str
    ) -> str:
        req = urllib.request.Request(checksum_url)
        r = urllib.request.urlopen(req).read()
        data = r.decode("utf-8").split("\n")

        hash = None
        checksum = None
        for line in data:
            text = line.split()
            if len(text) == 2 and "Hash" in text[0]:
                hash = text[1]

            if len(text) == 2 and filename in text[1]:
                checksum = text[0]
                break

        if not hash or not checksum:
            raise DotNetBadReleaseDataError(version)

        return "{}/{}".format(hash.lower(), checksum)

    def env(self, root):
        # Update the PATH only during the Build and Install step
        if root == self.installdir:
            return ["PATH={}:$PATH".format(self._dotnet_sdk_dir)]
        else:
            return []
