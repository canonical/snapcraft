# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2018-2020 Canonical Ltd
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

"""The ant plugin is useful for ant based parts.

The ant build system is commonly used to build Java projects.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - ant-properties:
      (object)
      A dictionary of key-value pairs. Set the following properties when
      running ant.

    - ant-build-targets:
      (list of strings)
      Run the given ant targets.

    - ant-channel:
      (string, default: latest/stable)
      The channel to use for ant in the snap store, if not using tarballs from
      the ant archive (see ant-version and ant-version-checksum).

    - ant-version:
      (string)
      The version of ant you want to use to build the source artifacts, if
      not using the snap version of ant.

    - ant-version-checksum:
      (string)
      The checksum for ant-version in the form of <digest-type>/<digest>.
      As an example "sha512/2a803f578f341e164f6753e410413d16ab60fab...".

    - ant-openjdk-version:
      (string)
      openjdk version available to the base to use. If not set the latest
      version available to the base will be used.

    - ant-buildfile
      (string, default: build.xml)
      The path to the Ant buildfile to use, relative to the root of the
      source tree.
"""

import logging
import os
from glob import glob
from typing import Sequence
from urllib.parse import urlsplit

from snapcraft import formatting_utils
from snapcraft.internal import errors, sources
from snapcraft.plugins.v1 import PluginV1

logger = logging.getLogger(__name__)

_ANT_ARCHIVE_FORMAT_URL = (
    "https://archive.apache.org/dist/ant/binaries/apache-ant-{version}-bin.tar.bz2"
)
_DEFAULT_ANT_SNAP_CHANNEL = "latest/stable"


class UnsupportedJDKVersionError(errors.SnapcraftError):

    fmt = (
        "The ant-openjdk-version plugin property was set to {version!r}.\n"
        "Valid values for the {base!r} base are: {valid_versions}."
    )

    def __init__(
        self, *, base: str, version: str, valid_versions: Sequence[str]
    ) -> None:
        super().__init__(
            base=base,
            version=version,
            valid_versions=formatting_utils.humanize_list(
                valid_versions, conjunction="or"
            ),
        )


class AntPlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["ant-properties"] = {"type": "object", "default": {}}

        schema["properties"]["ant-build-targets"] = {
            "type": "array",
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["ant-channel"] = {"type": "string"}

        schema["properties"]["ant-version"] = {"type": "string"}

        schema["properties"]["ant-version-checksum"] = {"type": "string"}

        schema["properties"]["ant-openjdk-version"] = {"type": "string", "default": ""}

        schema["properties"]["ant-buildfile"] = {"type": "string"}

        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return [
            "ant-channel",
            "ant-version",
            "ant-version-checksum",
            "ant-openjdk-version",
        ]

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["ant-build-targets", "ant-properties", "ant-buildfile"]

    @property
    def _ant_tar(self):
        if self._ant_tar_handle is None:
            ant_uri = _ANT_ARCHIVE_FORMAT_URL.format(version=self._ant_version)
            self._ant_tar_handle = sources.Tar(
                ant_uri, self._ant_dir, source_checksum=self._ant_checksum
            )
        return self._ant_tar_handle

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._setup_ant()
        self._setup_base_tools(project._get_build_base())

    def _setup_base_tools(self, base):
        if base in ("core", "core16"):
            valid_versions = ["8", "9"]
        elif base == "core18":
            valid_versions = ["8", "11"]

        version = self.options.ant_openjdk_version
        if not version:
            version = valid_versions[-1]
        elif version not in valid_versions:
            raise UnsupportedJDKVersionError(
                version=version, base=base, valid_versions=valid_versions
            )

        self.stage_packages.append("openjdk-{}-jre-headless".format(version))
        self.build_packages.append("openjdk-{}-jdk-headless".format(version))
        self._java_version = version

    def _setup_ant(self):
        if self.options.ant_version:
            self.use_archive_tarball = True
            self._ant_tar_handle = None
            self._ant_dir = os.path.join(self.partdir, "ant")
            self._ant_version = self.options.ant_version
            self._ant_checksum = self.options.ant_version_checksum
        else:
            self.use_archive_tarball = False

            if self.options.ant_channel:
                build_snap = "ant/" + self.options.ant_channel
            else:
                build_snap = "ant/" + _DEFAULT_ANT_SNAP_CHANNEL

            self.build_snaps.append(build_snap)

    def pull(self):
        super().pull()

        if self.use_archive_tarball:
            os.makedirs(self._ant_dir, exist_ok=True)
            self._ant_tar.download()

    def build(self):
        super().build()

        if self.use_archive_tarball:
            self._ant_tar.provision(
                self._ant_dir, clean_target=False, keep_tarball=True
            )

        command = ["ant"]

        if self.options.ant_buildfile:
            command.extend(["-f", self.options.ant_buildfile])

        if self.options.ant_build_targets:
            command.extend(self.options.ant_build_targets)

        for prop, value in self.options.ant_properties.items():
            command.extend(["-D{}={}".format(prop, value)])

        self.run(command, rootdir=self.builddir)
        files = glob(os.path.join(self.builddir, "target", "*.jar"))
        if files:
            jardir = os.path.join(self.installdir, "jar")
            os.makedirs(jardir)
            for f in files:
                base = os.path.basename(f)
                os.link(f, os.path.join(jardir, base))

        self._create_symlinks()

    def _create_symlinks(self):
        base = self.project._get_build_base()
        if base not in ("core18", "core16", "core"):
            raise errors.PluginBaseError(part_name=self.name, base=base)

        os.makedirs(os.path.join(self.installdir, "bin"), exist_ok=True)
        java_bin = glob(
            os.path.join(
                self.installdir,
                "usr",
                "lib",
                "jvm",
                "java-{}-openjdk-*".format(self._java_version),
                "bin",
                "java",
            )
        )[0]
        os.symlink(
            os.path.relpath(java_bin, os.path.join(self.installdir, "bin")),
            os.path.join(self.installdir, "bin", "java"),
        )

    def run(self, cmd, rootdir):
        super().run(cmd, cwd=rootdir, env=self._build_environment())

    def get_proxy_options(self, scheme):
        proxy = os.environ.get("{}_proxy".format(scheme))
        if proxy:
            parsed = urlsplit(proxy)
            if parsed.hostname is not None:
                yield "-D{}.proxyHost={}".format(scheme, parsed.hostname)
            if parsed.port is not None:
                yield "-D{}.proxyPort={}".format(scheme, parsed.port)
            if parsed.username is not None:
                yield "-D{}.proxyUser={}".format(scheme, parsed.username)
            if parsed.password is not None:
                yield "-D{}.proxyPassword={}".format(scheme, parsed.password)

    def _build_environment(self):
        env = os.environ.copy()

        if self.use_archive_tarball:
            ant_bin = os.path.join(self._ant_dir, "bin")
        else:
            ant_bin = os.path.join("snap", "bin", "ant")

        if env.get("PATH"):
            new_path = "{}:{}".format(ant_bin, env.get("PATH"))
        else:
            new_path = ant_bin

        env["PATH"] = new_path

        # Getting ant to use a proxy requires a little work; the JRE doesn't
        # help as much as it should.  (java.net.useSystemProxies=true ought
        # to do the trick, but it relies on desktop configuration rather
        # than using the standard environment variables.)
        ant_opts = []
        ant_opts.extend(self.get_proxy_options("http"))
        ant_opts.extend(self.get_proxy_options("https"))
        if ant_opts:
            env["ANT_OPTS"] = " ".join(opt.replace("'", "'\\''") for opt in ant_opts)

        return env

    def env(self, root):
        env = super().env(root)
        jars = glob(os.path.join(self.installdir, "jar", "*.jar"))
        if jars:
            jars = [
                os.path.join(root, "jar", os.path.basename(x)) for x in sorted(jars)
            ]
            env.extend(["CLASSPATH={}:$CLASSPATH".format(":".join(jars))])
        return env
