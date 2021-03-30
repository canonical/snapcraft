# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2020 Canonical Ltd
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

"""The godeps plugin can be used for godeps-enabled go projects.

These projects have a file containing information about the project's
dependencies. This file is typically called "dependencies.tsv," but may be
named anything.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

This plugin only works with go < 1.16.  As godeps is no longer maintained,
it is not recommended to continue using this plugin.

Additionally, this plugin uses the following plugin-specific keywords:

    - go-channel:
      (string, default: 1.15/stable)
      The Snap Store channel to install go from. If set to an empty string,
      go will be installed using the system's traditional package manager.

    - go-packages:
      (list of strings)
      Go packages to build/install, these must be a "main" package.
      Dependencies should have already been retrieved by the `godeps-file`
      used for this part.
      Packages that are not "main" will not cause an error, but would
      not be useful either.

    - godeps-file:
      (string)
      Path to the godeps dependencies file contained within the source
      (default: dependencies.tsv)

    - go-importpath:
      (string)
      This entry tells the checked out `source` to live within a certain path
      within `GOPATH`. This is required in order to work with absolute imports
      and import path checking.
"""

import logging
import os
import shutil

from snapcraft import common
from snapcraft.plugins.v1 import PluginV1

logger = logging.getLogger(__name__)


class GodepsPlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["go-channel"] = {
            "type": "string",
            "default": "1.15/stable",
        }
        schema["properties"]["godeps-file"] = {
            "type": "string",
            "default": "dependencies.tsv",
        }
        schema["properties"]["go-importpath"] = {"type": "string"}
        schema["properties"]["go-packages"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        # The import path must be specified.
        schema["required"] = ["source", "go-importpath"]

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["go-packages"]

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ["go-channel", "godeps-file", "go-importpath"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._setup_base_tools(options.go_channel)

        self._gopath = os.path.join(self.partdir, "go")
        self._gopath_src = os.path.join(self._gopath, "src")
        self._gopath_bin = os.path.join(self._gopath, "bin")
        self._gopath_pkg = os.path.join(self._gopath, "pkg")

    def _setup_base_tools(self, go_channel):
        if go_channel:
            self.build_snaps.append("go/{}".format(go_channel))
        else:
            self.build_packages.append("golang-go")

    def _install_godeps(self) -> None:
        env = self._build_environment()
        self.run(
            ["go", "get", "-d", "github.com/rogpeppe/godeps"],
            cwd=self._gopath_src,
            env=env,
        )

        # Chicken and egg - godeps itself has dependency requirements, otherwise
        # newer versions of go-toml will fail to build on older golang versions.
        gotoml_path = os.path.join(self._gopath_src, "github.com/pelletier/go-toml")
        self.run(
            ["git", "checkout", "4e9e0ee19b60b13eb79915933f44d8ed5f268bdd"],
            cwd=gotoml_path,
            env=env,
        )

        gotool_path = os.path.join(self._gopath_src, "github.com/kisielk/gotool")
        self.run(
            ["git", "checkout", "d6ce6262d87e3a4e153e86023ff56ae771554a41"],
            cwd=gotool_path,
            env=env,
        )

        gotools_path = os.path.join(self._gopath_src, "golang.org/x/tools")
        self.run(
            ["git", "checkout", "1937f90a1bb43667aff4059b1bab13eb15121e8e"],
            cwd=gotools_path,
            env=env,
        )

        self.run(
            ["go", "install", "github.com/rogpeppe/godeps"],
            cwd=self._gopath_src,
            env=env,
        )

    def pull(self):
        super().pull()

        path_in_gopath = os.path.join(self._gopath_src, self.options.go_importpath)
        os.makedirs(os.path.dirname(path_in_gopath), exist_ok=True)
        if os.path.islink(path_in_gopath):
            os.unlink(path_in_gopath)
        os.symlink(self.sourcedir, path_in_gopath)

        # Fetch and run godeps
        logger.info("Fetching godeps...")
        self._install_godeps()

        logger.info("Obtaining project dependencies...")
        self._run(
            [
                "godeps",
                "-t",
                "-u",
                os.path.join(self.sourcedir, self.options.godeps_file),
            ]
        )

    def clean_pull(self):
        super().clean_pull()

        # Remove the gopath (if present)
        if os.path.exists(self._gopath):
            shutil.rmtree(self._gopath)

    def build(self):
        super().build()

        for go_package in self.options.go_packages:
            self._run(["go", "install", go_package])
        if not self.options.go_packages:
            self._run(["go", "install", "./{}/...".format(self.options.go_importpath)])

        install_bin_path = os.path.join(self.installdir, "bin")
        os.makedirs(install_bin_path, exist_ok=True)
        os.makedirs(self._gopath_bin, exist_ok=True)
        for binary in os.listdir(self._gopath_bin):
            # Skip godeps. It serves no purpose in production.
            if binary == "godeps":
                continue

            binary_path = os.path.join(self._gopath_bin, binary)
            shutil.copy2(binary_path, install_bin_path)

    def clean_build(self):
        super().clean_build()

        if os.path.isdir(self._gopath_bin):
            shutil.rmtree(self._gopath_bin)

        if os.path.isdir(self._gopath_pkg):
            shutil.rmtree(self._gopath_pkg)

    def _run(self, cmd, **kwargs):
        env = self._build_environment()
        return self.run(cmd, cwd=self._gopath_src, env=env, **kwargs)

    def _build_environment(self):
        env = os.environ.copy()
        env["GOPATH"] = self._gopath
        env["GOBIN"] = self._gopath_bin

        # Add $GOPATH/bin so godeps is actually callable.
        env["PATH"] = "{}:{}".format(
            os.path.join(self._gopath, "bin"), env.get("PATH", "")
        )

        include_paths = []
        for root in [self.installdir, self.project.stage_dir]:
            include_paths.extend(
                common.get_library_paths(root, self.project.arch_triplet)
            )

        flags = common.combine_paths(include_paths, "-L", " ")
        env["CGO_LDFLAGS"] = "{} {} {}".format(
            env.get("CGO_LDFLAGS", ""), flags, env.get("LDFLAGS", "")
        )

        return env
