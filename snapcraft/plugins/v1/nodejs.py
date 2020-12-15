# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

"""The nodejs plugin is useful for node/npm based parts.

The plugin uses node to install dependencies from `package.json`. It
also sets up binaries defined in `package.json` into the `PATH`.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - nodejs-version:
      (string)
      The version of nodejs you want the snap to run on.
      This includes npm, as would be downloaded from https://nodejs.org
      Defaults to the current LTS release.

    - nodejs-package-manager
      (string; default: yarn)
      The language package manager to use to drive installation
      of node packages. Can be either `npm` or `yarn` (default).

    - nodejs-yarn-version:
      (string)
      Applicable when using yarn. Defaults to the latest if not set.
"""

import collections
import contextlib
import json
import os
import shutil
import subprocess
import sys

from snapcraft import sources
from snapcraft.file_utils import link_or_copy, link_or_copy_tree
from snapcraft.internal import errors
from snapcraft.plugins.v1 import PluginV1

_NODEJS_BASE = "node-v{version}-linux-{arch}"
_NODEJS_VERSION = "8.12.0"
_NODEJS_TMPL = "https://nodejs.org/dist/v{version}/{base}.tar.gz"
_NODEJS_ARCHES = {
    "i386": "x86",
    "amd64": "x64",
    "armhf": "armv7l",
    "arm64": "arm64",
    "ppc64el": "ppc64le",
    "s390x": "s390x",
}
_YARN_LATEST_URL = "https://yarnpkg.com/latest.tar.gz"
# e.g.; https://github.com/yarnpkg/yarn/releases/download/v1.12.0/yarn-v1.12.0.tar.gz
_YARN_VERSION_URL = (
    "https://github.com/yarnpkg/yarn/releases/download/{version}/yarn-{version}.tar.gz"
)


class NodejsPluginMissingPackageJsonError(errors.SnapcraftError):

    fmt = (
        "Could not find a 'package.json' in the source tree.\n"
        "Verify that a 'package.json' exists at the root of the source tree\n"
        "or consider using the 'source-subdir' property if it is located in a subdirectory."
    )


class NodePlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["nodejs-version"] = {
            "type": "string",
            "default": _NODEJS_VERSION,
        }
        schema["properties"]["nodejs-package-manager"] = {
            "type": "string",
            "default": "yarn",
            "enum": ["npm", "yarn"],
        }
        schema["properties"]["nodejs-yarn-version"] = {"type": "string", "default": ""}

        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["nodejs-version", "nodejs-package-manager", "nodejs-yarn-version"]

    @property
    def _nodejs_tar(self):
        if self._nodejs_tar_handle is None:
            self._nodejs_tar_handle = sources.Tar(
                self._nodejs_release_uri, self._npm_dir
            )
        return self._nodejs_tar_handle

    @property
    def _yarn_tar(self):
        if self._yarn_tar_handle is None:
            if self.options.nodejs_yarn_version:
                url = _YARN_VERSION_URL.format(version=self.options.nodejs_yarn_version)
            else:
                url = _YARN_LATEST_URL
            self._yarn_tar_handle = sources.Tar(url, self._npm_dir)
        return self._yarn_tar_handle

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._npm_dir = os.path.join(self.partdir, "npm")

        self._manifest = collections.OrderedDict()

        self._nodejs_release_uri = get_nodejs_release(
            self.options.nodejs_version, self.project.deb_arch
        )
        self._nodejs_tar_handle = None
        self._yarn_tar_handle = None

    def pull(self):
        super().pull()

        os.makedirs(self._npm_dir, exist_ok=True)
        self._nodejs_tar.download()
        if self.options.nodejs_package_manager == "yarn":
            self._yarn_tar.download()

        # install node and yarn.
        self._install_node_and_yarn(rootdir=self.sourcedir)

    def clean_pull(self):
        super().clean_pull()
        # Remove the npm directory (if any)
        if os.path.exists(self._npm_dir):
            shutil.rmtree(self._npm_dir)

    def build(self):
        super().build()

        package_dir = self._install(rootdir=self.builddir)

        # Now move everything over to the plugin's installdir
        link_or_copy_tree(package_dir, self.installdir)
        # Copy in the node binary
        link_or_copy(
            os.path.join(self._npm_dir, "bin", "node"),
            os.path.join(self.installdir, "bin", "node"),
        )
        # Create binary entries
        package_json = self._get_package_json(rootdir=self.builddir)
        _create_bins(package_json, self.installdir)

        lock_file_path = os.path.join(self.installdir, "yarn.lock")
        if os.path.isfile(lock_file_path):
            with open(lock_file_path) as lock_file:
                self._manifest["yarn-lock-contents"] = lock_file.read()

        # Get the names and versions of installed packages
        if self.options.nodejs_package_manager == "npm":
            installed_node_packages = self._get_installed_node_packages(self.installdir)
            self._manifest["node-packages"] = [
                "{}={}".format(name, installed_node_packages[name])
                for name in installed_node_packages
            ]
        # Skip this step if yarn is used, as it may produce different
        # dependency trees than npm
        else:
            self._manifest["node-packages"] = []

    def _install_node_and_yarn(self, rootdir):
        self._nodejs_tar.provision(self._npm_dir, clean_target=False, keep_tarball=True)
        if self.options.nodejs_package_manager == "yarn":
            self._yarn_tar.provision(
                self._npm_dir, clean_target=False, keep_tarball=True
            )
        # Check to see if package.json exists in pull step
        self._get_package_json(rootdir)

    def _install(self, rootdir):
        cmd = [os.path.join(self._npm_dir, "bin", self.options.nodejs_package_manager)]

        if self.options.nodejs_package_manager == "yarn":
            if os.getenv("http_proxy"):
                cmd.extend(["--proxy", os.getenv("http_proxy")])
            if os.getenv("https_proxy"):
                cmd.extend(["--https-proxy", os.getenv("https_proxy")])

        flags = []

        if self.options.nodejs_package_manager == "npm":
            flags.append("--unsafe-perm")

        # Run once to download dependencies and run install scripts
        self.run(cmd + ["install"] + flags, rootdir)

        package_json = self._get_package_json(rootdir)
        # Take into account scoped names
        name = package_json["name"].lstrip("@").replace("/", "-")
        version = package_json["version"]

        # npm pack will create a tarball of the form
        # <package-name>-<package-version>.tgz
        # and we will tell yarn pack to create one with this
        # predictable name.
        package_tar_path = "{name}-{version}.tgz".format(name=name, version=version)
        if self.options.nodejs_package_manager == "yarn":
            self.run(cmd + ["pack", "--filename", package_tar_path], rootdir)
        else:
            self.run(cmd + ["pack"], rootdir)

        package_dir = os.path.join(rootdir, "package")
        package_tar = sources.Tar(package_tar_path, rootdir)
        package_tar.file = package_tar_path
        os.makedirs(package_dir, exist_ok=True)
        package_tar.provision(package_dir)

        with contextlib.suppress(FileNotFoundError):
            shutil.copy(
                os.path.join(rootdir, "yarn.lock"),
                os.path.join(package_dir, "yarn.lock"),
            )

        flags.append("--offline")
        flags.append("--prod")

        self.run(cmd + ["install"] + flags, package_dir)

        return package_dir

    def run(self, cmd, rootdir):
        super().run(cmd, cwd=rootdir, env=self._build_environment())

    def run_output(self, cmd, rootdir):
        return super().run_output(cmd, cwd=rootdir, env=self._build_environment())

    def _build_environment(self):
        env = os.environ.copy()
        npm_bin = os.path.join(self._npm_dir, "bin")

        if env.get("PATH"):
            new_path = "{}:{}".format(npm_bin, env.get("PATH"))
        else:
            new_path = npm_bin

        # npm has the behavior that, if it detects that SUDO_UID is set,
        # it will then set the uid of some of its child processes (such as
        # git when installing a package that specifies a git repository)
        # to that of the sudoer, ignoring --unsafe-perm entirely.
        # We have to unset SUDO_UID and SUDO_GID to prevent this.

        env.pop("SUDO_UID", None)
        env.pop("SUDO_GID", None)

        env["PATH"] = new_path
        return env

    def _get_package_json(self, rootdir):
        try:
            with open(os.path.join(rootdir, "package.json")) as json_file:
                return json.load(json_file)
        except FileNotFoundError as not_found_error:
            raise NodejsPluginMissingPackageJsonError() from not_found_error

    def _get_installed_node_packages(self, cwd):
        # There is no yarn ls
        cmd = [os.path.join(self._npm_dir, "bin", "npm"), "ls", "--json"]
        try:
            output = self.run_output(cmd, cwd)
        except subprocess.CalledProcessError as error:
            # XXX When dependencies have missing dependencies, an error like
            # this is printed to stderr:
            # npm ERR! peer dep missing: glob@*, required by glob-promise@3.1.0
            # retcode is not 0, which raises an exception.
            output = error.output.decode(sys.getfilesystemencoding()).strip()
        packages = collections.OrderedDict()
        output_json = json.loads(output, object_pairs_hook=collections.OrderedDict)
        dependencies = output_json.get("dependencies", [])
        while dependencies:
            key, value = dependencies.popitem(last=False)
            # XXX Just as above, dependencies without version are the ones
            # missing.
            if "version" in value:
                packages[key] = value["version"]
            if "dependencies" in value:
                dependencies.update(value["dependencies"])
        return packages

    def get_manifest(self):
        return self._manifest


def _create_bins(package_json, directory):
    bin_entry = package_json.get("bin")
    if not bin_entry:
        return

    bin_dir = os.path.join(directory, "bin")
    os.makedirs(bin_dir, exist_ok=True)

    if type(bin_entry) == dict:
        binaries = bin_entry
    elif type(bin_entry) == str:
        # Support for scoped names of the form of @org/name
        name = package_json["name"]
        binaries = {name[name.find("/") + 1 :]: bin_entry}
    else:
        raise errors.SnapcraftEnvironmentError(
            "The plugin is not prepared to handle bin entries of "
            "type {!r}".format(type(bin_entry))
        )

    for bin_name, bin_path in binaries.items():
        target = os.path.join(bin_dir, bin_name)
        # The binary might be already created from upstream sources.
        if os.path.exists(os.path.join(target)):
            continue
        source = os.path.join("..", bin_path)
        os.symlink(source, target)
        # Make it executable
        os.chmod(os.path.realpath(target), 0o755)


def _get_nodejs_base(node_engine, machine):
    if machine not in _NODEJS_ARCHES:
        raise errors.SnapcraftEnvironmentError(
            "architecture not supported ({})".format(machine)
        )
    return _NODEJS_BASE.format(version=node_engine, arch=_NODEJS_ARCHES[machine])


def get_nodejs_release(node_engine, arch):
    return _NODEJS_TMPL.format(
        version=node_engine, base=_get_nodejs_base(node_engine, arch)
    )
