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

"""This plugin is useful for building parts that use maven.

The maven build system is commonly used to build Java projects.
The plugin requires a pom.xml in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - maven-options:
      (list of strings)
      Flags to pass to the build using the maven semantics for parameters.

    - maven-targets:
      (list of strings)
      List of target directories where the built jars are stored.
      It corresponds to the <build><directory>target</directory></build>
      from the Maven's build declaration.

    - maven-version:
      (string)
      The version of maven you want to use to build the source artifacts.
      Defaults to the current release downloadable from
      https://archive.apache.org/dist/maven/maven-3/.

    - maven-version-checksum:
      (string)
      The checksum for maven-version in the form of <digest-type>/<digest>.
      As an example "sha512/2a803f578f341e164f6753e410413d16ab60fab...".

    - maven-openjdk-version:
      (string)
      openjdk version available to the base to use. If not set the latest
      version available to the base will be used.
"""

import logging
import os
import shutil
import textwrap
from glob import glob
from typing import Optional, Sequence
from urllib.parse import urlparse
from xml.etree import ElementTree

from snapcraft import file_utils, formatting_utils
from snapcraft.internal import errors, sources
from snapcraft.plugins.v1 import PluginV1

logger = logging.getLogger(__name__)

_DEFAULT_MAVEN_VERSION = "3.5.4"
_DEFAULT_MAVEN_CHECKSUM = "sha512/2a803f578f341e164f6753e410413d16ab60fabe31dc491d1fe35c984a5cce696bc71f57757d4538fe7738be04065a216f3ebad4ef7e0ce1bb4c51bc36d6be86"
_MAVEN_URL = "https://archive.apache.org/dist/maven/maven-3/{version}/binaries/apache-maven-{version}-bin.tar.gz"


class EmptyBuildTargetDirectoryError(errors.SnapcraftException):
    def __init__(self, target_directory: str) -> None:
        self._target_directory = target_directory

    def get_details(self) -> Optional[str]:
        return textwrap.dedent(
            """\
            This could happen if your `maven-targets` points to a directory that is not the same as the one
            in your Maven's <build><directory>target</directory></build> declaration. By default, Snap's
            Maven plugin won't need `maven-targets` declaration as it will work with the default Maven behavior.
            If you are not sure, try removing `maven-targets` definition and build your Snap again."""
        )

    def get_brief(self) -> str:
        return "Could not find built jar files for part in the following directory: {!r}".format(
            self._target_directory
        )

    def get_resolution(self) -> str:
        return textwrap.dedent(
            """\
            `maven-targets` should point the same directory as Maven's
            <build><directory>target</directory></build> declaration."""
        )


class UnsupportedJDKVersionError(errors.SnapcraftError):
    fmt = (
        "The maven-openjdk-version plugin property was set to {version!r}.\n"
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


class MavenPlugin(PluginV1):
    @classmethod
    def schema(cls):

        schema = super().schema()

        schema["properties"]["maven-options"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["maven-targets"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [""],
        }

        schema["properties"]["maven-version"] = {"type": "string"}

        schema["properties"]["maven-version-checksum"] = {"type": "string"}

        schema["properties"]["maven-openjdk-version"] = {
            "type": "string",
            "default": "",
        }

        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ["maven-version", "maven-version-checksum", "maven-openjdk-version"]

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["maven-options", "maven-targets"]

    @property
    def _maven_tar(self):
        if self._maven_tar_handle is None:
            maven_uri = _MAVEN_URL.format(version=self._maven_version)
            self._maven_tar_handle = sources.Tar(
                maven_uri, self._maven_dir, source_checksum=self._maven_checksum
            )
        return self._maven_tar_handle

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._setup_maven()
        self._setup_base_tools(project._get_build_base())

    def _setup_base_tools(self, base):
        if base in ("core", "core16"):
            valid_versions = ["8", "9"]
        elif base == "core18":
            valid_versions = ["8", "11"]

        version = self.options.maven_openjdk_version
        if not version:
            version = valid_versions[-1]
        elif version not in valid_versions:
            raise UnsupportedJDKVersionError(
                version=version, base=base, valid_versions=valid_versions
            )

        self.stage_packages.append("openjdk-{}-jre-headless".format(version))
        self.build_packages.append("openjdk-{}-jdk-headless".format(version))
        self._java_version = version

    def _setup_maven(self):
        self._maven_tar_handle = None
        self._maven_dir = os.path.join(self.partdir, "maven")
        if self.options.maven_version:
            self._maven_version = self.options.maven_version
            self._maven_checksum = self.options.maven_version_checksum
        else:
            self._maven_version = _DEFAULT_MAVEN_VERSION
            self._maven_checksum = _DEFAULT_MAVEN_CHECKSUM

    def _use_proxy(self):
        return any(k in os.environ for k in ("http_proxy", "https_proxy"))

    def pull(self):
        super().pull()

        os.makedirs(self._maven_dir, exist_ok=True)
        self._maven_tar.download()

    def clean_pull(self):
        super().clean_pull()
        # Remove the maven directory (if any)
        if os.path.exists(self._maven_dir):
            shutil.rmtree(self._maven_dir)

    def build(self):
        super().build()

        self._maven_tar.provision(
            self._maven_dir, clean_target=False, keep_tarball=True
        )

        mvn_cmd = ["mvn", "package"]
        if self._use_proxy():
            settings_path = os.path.join(self.partdir, "m2", "settings.xml")
            _create_settings(settings_path)
            mvn_cmd += ["-s", settings_path]

        self.run(mvn_cmd + self.options.maven_options, rootdir=self.builddir)

        for f in self.options.maven_targets:
            src = os.path.join(self.builddir, f, "target")
            jarfiles = glob(os.path.join(src, "*.jar"))
            warfiles = glob(os.path.join(src, "*.war"))
            arfiles = glob(os.path.join(src, "*.[jw]ar"))

            if len(arfiles) == 0:
                raise EmptyBuildTargetDirectoryError(src)
            if len(jarfiles) > 0 and len(f) == 0:
                basedir = "jar"
            elif len(warfiles) > 0 and len(f) == 0:
                basedir = "war"
            else:
                basedir = f

            targetdir = os.path.join(self.installdir, basedir)
            os.makedirs(targetdir, exist_ok=True)
            for src in arfiles:
                # Make rebuilding easier when enabled
                base = os.path.basename(src)
                dst = os.path.join(targetdir, base)
                if os.path.exists(dst):
                    os.unlink(dst)
                file_utils.link_or_copy(src, dst)

        self._create_symlinks()

    def _create_symlinks(self):
        if self.project._get_build_base() not in ("core18", "core16", "core"):
            raise errors.PluginBaseError(
                part_name=self.name, base=self.project._get_build_base()
            )

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

    def _build_environment(self):
        env = os.environ.copy()
        maven_bin = os.path.join(self._maven_dir, "bin")

        if env.get("PATH"):
            new_path = "{}:{}".format(maven_bin, env.get("PATH"))
        else:
            new_path = maven_bin

        env["PATH"] = new_path
        return env


def _create_settings(settings_path):
    settings = ElementTree.Element(
        "settings",
        attrib={
            "xmlns": "http://maven.apache.org/SETTINGS/1.0.0",
            "xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
            "xsi:schemaLocation": (
                "http://maven.apache.org/SETTINGS/1.0.0 "
                "http://maven.apache.org/xsd/settings-1.0.0.xsd"
            ),
        },
    )
    element = ElementTree.Element("interactiveMode")
    element.text = "false"
    settings.append(element)
    proxies = ElementTree.Element("proxies")
    for protocol in ("http", "https"):
        env_name = "{}_proxy".format(protocol)
        if env_name not in os.environ:
            continue
        proxy_url = urlparse(os.environ[env_name])
        proxy = ElementTree.Element("proxy")
        proxy_tags = [
            ("id", env_name),
            ("active", "true"),
            ("protocol", protocol),
            ("host", proxy_url.hostname),
            ("port", str(proxy_url.port)),
        ]
        if proxy_url.username is not None:
            proxy_tags.extend(
                [("username", proxy_url.username), ("password", proxy_url.password)]
            )
        proxy_tags.append(("nonProxyHosts", _get_no_proxy_string()))
        for tag, text in proxy_tags:
            element = ElementTree.Element(tag)
            element.text = text
            proxy.append(element)
        proxies.append(proxy)
    settings.append(proxies)
    tree = ElementTree.ElementTree(settings)
    os.makedirs(os.path.dirname(settings_path), exist_ok=True)
    with open(settings_path, "w") as f:
        tree.write(f, encoding="unicode")
        f.write("\n")


def _get_no_proxy_string():
    no_proxy = [k.strip() for k in os.environ.get("no_proxy", "localhost").split(",")]
    return "|".join(no_proxy)
