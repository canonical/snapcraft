# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

"""The go plugin can be used for go projects using `go get`.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - go-packages:
      (list of strings)
      Go packages to fetch, these must be a "main" package. Dependencies
      are pulled in automatically by `go get`.
      Packages that are not "main" will not cause an error, but would
      not be useful either.
      If the package is a part of the go-importpath the local package
      corresponding to those sources will be used.

    - go-importpath:
      (string)
      This entry tells the checked out `source` to live within a certain path
      within `GOPATH`.
      This is not needed and does not affect `go-packages`.

    - go-buildtags:
      (list of strings)
      Tags to use during the go build. Default is not to use any build tags.
"""

import logging
import os
import shutil
from glob import iglob

import snapcraft
from snapcraft import common


logger = logging.getLogger(__name__)


class GoPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["go-packages"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["go-importpath"] = {"type": "string", "default": ""}
        schema["properties"]["go-buildtags"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        if "required" in schema:
            del schema["required"]

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["go-packages", "go-buildtags"]

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ["go-packages"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        if "go" not in self.build_snaps and not any(
            (s.startswith("go/") for s in self.build_snaps)
        ):
            self.build_packages.append("golang-go")
        self._gopath = os.path.join(self.partdir, "go")
        self._gopath_src = os.path.join(self._gopath, "src")
        self._gopath_bin = os.path.join(self._gopath, "bin")
        self._gopath_pkg = os.path.join(self._gopath, "pkg")

    def pull(self):
        # use -d to only download (build will happen later)
        # use -t to also get the test-deps
        # since we are not using -u the sources will stick to the
        # original checkout.
        super().pull()
        os.makedirs(self._gopath_src, exist_ok=True)

        if any(iglob("{}/**/*.go".format(self.sourcedir), recursive=True)):
            go_package = self._get_local_go_package()
            go_package_path = os.path.join(self._gopath_src, go_package)
            if os.path.islink(go_package_path):
                os.unlink(go_package_path)
            os.makedirs(os.path.dirname(go_package_path), exist_ok=True)
            os.symlink(self.sourcedir, go_package_path)
            self._run(["go", "get", "-t", "-d", "./{}/...".format(go_package)])

        for go_package in self.options.go_packages:
            self._run(["go", "get", "-t", "-d", go_package])

    def clean_pull(self):
        super().clean_pull()

        # Remove the gopath (if present)
        if os.path.exists(self._gopath):
            shutil.rmtree(self._gopath)

    def _get_local_go_package(self):
        if self.options.go_importpath:
            go_package = self.options.go_importpath
        else:
            logger.warning(
                "Please consider setting `go-importpath` for the {!r} "
                "part".format(self.name)
            )
            go_package = os.path.basename(os.path.abspath(self.options.source))
        return go_package

    def _get_local_main_packages(self):
        search_path = "./{}/...".format(self._get_local_go_package())
        packages = self._run_output(
            ["go", "list", "-f", "{{.ImportPath}} {{.Name}}", search_path]
        )
        packages_split = [p.split() for p in packages.splitlines()]
        main_packages = [p[0] for p in packages_split if p[1] == "main"]
        return main_packages

    def build(self):
        super().build()

        tags = []
        if self.options.go_buildtags:
            tags = ["-tags={}".format(",".join(self.options.go_buildtags))]

        packages = self.options.go_packages
        if not packages:
            packages = self._get_local_main_packages()
        for package in packages:
            binary = os.path.join(self._gopath_bin, self._binary_name(package))
            self._run(["go", "build", "-o", binary] + tags + [package])

        install_bin_path = os.path.join(self.installdir, "bin")
        os.makedirs(install_bin_path, exist_ok=True)
        for binary in os.listdir(self._gopath_bin):
            binary_path = os.path.join(self._gopath_bin, binary)
            shutil.copy2(binary_path, install_bin_path)

    def _binary_name(self, package):
        package = package.replace("/...", "")
        return package.split("/")[-1]

    def clean_build(self):
        super().clean_build()

        if os.path.isdir(self._gopath_bin):
            shutil.rmtree(self._gopath_bin)

        if os.path.isdir(self._gopath_pkg):
            shutil.rmtree(self._gopath_pkg)

    def _run(self, cmd, **kwargs):
        env = self._build_environment()
        return self.run(cmd, cwd=self._gopath_src, env=env, **kwargs)

    def _run_output(self, cmd, **kwargs):
        env = self._build_environment()
        return self.run_output(cmd, cwd=self._gopath_src, env=env, **kwargs)

    def _build_environment(self):
        env = os.environ.copy()
        env["GOPATH"] = self._gopath
        env["GOBIN"] = self._gopath_bin

        include_paths = []
        for root in [self.installdir, self.project.stage_dir]:
            include_paths.extend(
                common.get_library_paths(root, self.project.arch_triplet)
            )

        flags = common.combine_paths(include_paths, "-L", " ")
        env["CGO_LDFLAGS"] = "{} {} {}".format(
            env.get("CGO_LDFLAGS", ""), flags, env.get("LDFLAGS", "")
        )

        if self.project.is_cross_compiling:
            env["CC"] = "{}-gcc".format(self.project.arch_triplet)
            env["CXX"] = "{}-g++".format(self.project.arch_triplet)
            env["CGO_ENABLED"] = "1"
            # See https://golang.org/doc/install/source#environment
            go_archs = {"armhf": "arm", "i386": "386", "ppc64el": "ppc64le"}
            env["GOARCH"] = go_archs.get(self.project.deb_arch, self.project.deb_arch)
            if self.project.deb_arch == "armhf":
                env["GOARM"] = "7"
        return env

    def enable_cross_compilation(self):
        pass
