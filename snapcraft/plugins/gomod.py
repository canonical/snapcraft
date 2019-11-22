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

"""The gomod plugin can be used for go projects using modules.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - go-channel:
      (string, default: latest/stable)
      The Snap Store channel to install go from.

    - go-packages:
      (list of strings)
      Go packages to build, these must be a "main" package.  Packages
      that are not "main" will not cause an error, but would not be
      useful either.

    - go-buildtags:
      (list of strings)
      Tags to use during the go build. Default is not to use any build tags.

    - go-proxy:
      (list of strings)
      List of locations to use as the go module proxy. Go will attempt to
      download modules from each proxy in turn until one succeeds, if the
      value "direct" occurs in the list then go will attempt to fetch the
      module's source directly. Default is ["https://proxy.golang.org",
      "direct"].

    - go-no-proxy:
      (list of strings)
      List of module patterns that will be fetched directly rather than
      via the configured proxy. Each value is a glob pattern that is
      prefix matched to the required modules. Default is to not fetch
      any modules directly.

    - go-sumdb:
      (string)
      Location to use as the module checksum database. This
      takes one of the following formats: "<database-name>",
      "<database-name>+<public-key>", or "<database-name>+<public-key>
      <database-url>". The public-key is required if the database is not
      one of "sum.golang.org" or "sum.golang.google.cn". If not specified
      the database URL is assumed to be "https://<database-name>". This
      can be set to "off" to disable all module authentication. Default is
      "sum.golang.org".

    - go-no-sumdb:
      (list of strings)
      List of module patterns that will not be authenticated with the
      module checksum database. Each value is a glob pattern that is
      prefix matched to the required modules. Default is to authenticate
      all modules.

    - go-private:
      (list of strings)
      List of module patterns that are private, these modules will not
      be downloaded from a proxy, or authenticated against the checksum
      database. Each value is a glob pattern that is prefix matched to
      the required modules. Default is that there are no private modules.

"""


import os

import snapcraft


class GoModPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["go-buildtags"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["go-channel"] = {
            "type": "string",
            "default": "latest/stable",
        }
        schema["properties"]["go-no-proxy"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["go-no-sumdb"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["go-packages"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
        }
        schema["properties"]["go-private"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["go-proxy"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": ["https://proxy.golang.org", "direct"],
        }
        schema["properties"]["go-sumdb"] = {
            "type": "string",
            "default": "sum.golang.org",
        }
        schema["required"] = ["go-packages"]
        return schema

    @classmethod
    def get_build_properties(cls):
        return ["go-packages", "go-buildtags"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._env = [
            "GO111MODULE=on",
            "GOPROXY={}".format(",".join(options.go_proxy)),
            "GOSUMDB={}".format(options.go_sumdb),
        ]
        if options.go_no_proxy:
            self._env.append("GONOPROXY={}".format(",".join(options.go_no_proxy)))
        if options.go_no_sumdb:
            self._env.append("GONOSUMDB={}".format(",".join(options.go_no_sumdb)))
        if options.go_private:
            self._env.append("GOPRIVATE={}".format(",".join(options.go_private)))

        self.build_snaps.append("go/{}".format(options.go_channel))
        self.build_packages.append("gcc")

    def enable_cross_compilation(self):
        self.build_packages.extend(_arch_packages.get(self.project.deb_arch, []))
        self._env.extend(_arch_envs.get(self.project.deb_arch, []))

    def env(self, root):
        env = super().env(root)
        env.extend(self._env)
        return env

    def pull(self):
        super().pull()

        self.run(["go", "mod", "download"])

    def clean_pull(self):
        super().clean_pull()

        self.run(["go", "clean", "-modcache"])

    def build(self):
        super().build()

        for p in self.options.go_packages:
            cmd = ["go", "build", "-trimpath"]
            if self.options.go_buildtags:
                cmd.extend(["-tags", ",".join(self.options.go_buildtags)])
            cmd.extend(
                ["-o", os.path.join(self.installdir, "bin", p.split("/")[-1]), p]
            )

            self.run(cmd)

    def clean_build(self):
        super().clean_build()

        cmd = ["go", "clean"]
        cmd.extend(self.options.go_packages)
        self.run(cmd)


_arch_packages = {
    "amd64": ["gcc-x86-64-linux-gnu"],
    "arm64": ["gcc-aarch64-linux-gnu"],
    "armhf": ["gcc-arm-linux-gnueabihf"],
    "i386": ["gcc-i686-linux-gnu"],
    "ppc64el": ["gcc-powerpc64le-linux-gnu"],
    "powerpc": ["gcc-powerpc-linux-gnu"],
    "s390x": ["gcc-s390x-linux-gnu"],
}

_arch_envs = {
    "amd64": ["GOARCH=amd64"],
    "arm64": ["GOARCH=arm64"],
    "armhf": ["GOARCH=arm", "GOARM=7"],
    "i386": ["GOARCH=386"],
    "ppc64el": ["GOARCH=ppc64le"],
    "powerpc": ["GOARCH=ppc"],
    "s390x": ["GOARCH=s390x"],
}
