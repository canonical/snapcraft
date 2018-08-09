# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

"""The kbuild plugin is used for building kbuild based projects as snapcraft
parts.

This plugin is based on the snapcraft.BasePlugin and supports the properties
provided by that plus the following kbuild specific options with semantics as
explained above:

    - kdefconfig:
      (list of kdefconfigs)
      defconfig target to use as the base configuration. default: "defconfig"

    - kconfigfile:
      (filepath)
      path to file to use as base configuration. If provided this option wins
      over everything else. default: None

    - kconfigflavour
      (string)
      Ubuntu config flavour to use as base configuration. If provided this
      option wins over kdefconfig. default: None

    - kconfigs
      (list of strings)
      explicit list of configs to force; this will override the configs that
      were set as base through kdefconfig and kconfigfile and dependent configs
      will be fixed using the defaults encoded in the kbuild config
      definitions.  If you don't want default for one or more implicit configs
      coming out of these, just add them to this list as well.

The plugin applies your selected defconfig first by running

    make defconfig

and then uses the kconfigs flag to augment the resulting config by prepending
the configured kconfigs values to the .config and running

    "yes" "" | make oldconfig

to create an updated .config file.

If kconfigfile is provided this plugin will use the provided config file
wholesale as the starting point instead of make $kdefconfig. In case user
configures both a kdefconfig as well as kconfigfile, kconfigfile approach will
be used.
"""

import logging
import os
import subprocess
import re

from snapcraft import BasePlugin

import snapcraft

logger = logging.getLogger(__name__)


class KBuildPlugin(BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["kdefconfig"] = {"type": "array", "default": ["defconfig"]}

        schema["properties"]["kconfigfile"] = {"type": "string", "default": None}

        schema["properties"]["kconfigs"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["kconfigflavour"] = {"type": "string", "default": None}

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["kdefconfig", "kconfigfile", "kconfigs", "kconfigflavour"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(["bc", "gcc", "make"])

        self.make_targets = []
        self.make_install_targets = ["install"]
        self.make_cmd = ["make", "-j{}".format(self.parallel_build_count)]
        if logger.isEnabledFor(logging.DEBUG):
            self.make_cmd.append("V=1")

    def enable_cross_compilation(self):
        self.make_cmd.append("ARCH={}".format(self.project.kernel_arch))
        if os.environ.get("CROSS_COMPILE"):
            toolchain = os.environ["CROSS_COMPILE"]
        else:
            toolchain = self.project.cross_compiler_prefix
        self.make_cmd.append("CROSS_COMPILE={}".format(toolchain))

        env = os.environ.copy()
        self.make_cmd.append(
            "PATH={}:/usr/{}/bin".format(env.get("PATH", ""), self.project.arch_triplet)
        )

    def assemble_ubuntu_config(self, config_path):
        try:
            with open(os.path.join(self.sourcedir, "debian", "debian.env"), "r") as f:
                env = f.read()
        except OSError as e:
            raise RuntimeError("Unable to access {}: {}".format(e.filename, e.strerror))
        arch = self.project.deb_arch
        try:
            branch = env.split(".")[1].strip()
        except IndexError:
            raise RuntimeError("Malformed debian.env, cannot extract branch name")
        flavour = self.options.kconfigflavour

        configfiles = []
        baseconfigdir = os.path.join(
            self.sourcedir, "debian.{}".format(branch), "config"
        )
        archconfigdir = os.path.join(
            self.sourcedir, "debian.{}".format(branch), "config", arch
        )
        commonconfig = os.path.join(baseconfigdir, "config.common.ports")
        ubuntuconfig = os.path.join(baseconfigdir, "config.common.ubuntu")
        archconfig = os.path.join(archconfigdir, "config.common.{}".format(arch))
        flavourconfig = os.path.join(archconfigdir, "config.flavour.{}".format(flavour))
        configfiles.append(commonconfig)
        configfiles.append(ubuntuconfig)
        configfiles.append(archconfig)
        configfiles.append(flavourconfig)
        # assemble .config
        try:
            with open(config_path, "w") as config_file:
                for config_part_path in (
                    commonconfig,
                    ubuntuconfig,
                    archconfig,
                    flavourconfig,
                ):
                    with open(config_part_path) as config_part:
                        config_file.write(config_part.read())
        except OSError as e:
            raise RuntimeError(
                "Unable to access {!r}: {}".format(e.filename, e.strerror)
            )

    def get_config_path(self):
        return os.path.join(self.builddir, ".config")

    def do_base_config(self, config_path):
        # if the parts build dir already contains a .config file,
        # use it
        if os.path.isfile(config_path):
            return
        # if kconfigfile is provided use that
        # elif kconfigflavour is provided, assemble the ubuntu.flavour config
        # otherwise use defconfig to seed the base config
        if self.options.kconfigfile:
            # This file gets modified, no hard links here
            snapcraft.file_utils.copy(self.options.kconfigfile, config_path)
        elif self.options.kconfigflavour:
            self.assemble_ubuntu_config(config_path)
        else:
            # we need to run this with -j1, unit tests are a good defense here.
            make_cmd = self.make_cmd.copy()
            make_cmd[1] = "-j1"
            self.run(make_cmd + self.options.kdefconfig)

    def do_patch_config(self, config_path):
        # prepend the generated file with provided kconfigs
        #  - concat kconfigs to buffer
        #  - read current .config and append
        #  - write out to disk
        if not self.options.kconfigs:
            return

        config = "\n".join(self.options.kconfigs)

        # note that prepending and appending the overrides seems
        # only way to convince all kbuild versions to pick up the
        # configs during oldconfig in .config
        with open(config_path, "r") as f:
            config = "{config_override}\n\n{config}\n{config_override}\n".format(
                config_override=config, config=f.read()
            )

        with open(config_path, "w") as f:
            f.write(config)

    def do_remake_config(self):
        # update config to include kconfig amendments using oldconfig
        cmd = 'yes "" | {} oldconfig'.format(" ".join(self.make_cmd))
        subprocess.check_call(cmd, shell=True, cwd=self.builddir)

    def do_configure(self):
        config_path = self.get_config_path()

        self.do_base_config(config_path)
        self.do_patch_config(config_path)
        self.do_remake_config()

    def do_build(self):
        # Linux's kernel Makefile gets confused if it is invoked with the
        # environment setup by another Linux's Makefile:
        # linux/package/Makefile -> snapcraft -> linux/Makefile
        # fix the problem removing the offending make option (-I...)
        if "MAKEFLAGS" in os.environ:
            makeflags = re.sub("-I[\S]*", "", os.environ["MAKEFLAGS"])
            os.environ["MAKEFLAGS"] = makeflags
        # build the software
        self.run(self.make_cmd + self.make_targets)

    def do_install(self):
        # install to installdir
        self.run(
            self.make_cmd
            + ["CONFIG_PREFIX={}".format(self.installdir)]
            + self.make_install_targets
        )

    def build(self):
        super().build()

        self.do_configure()
        self.do_build()
        if "no-install" not in self.options.build_attributes:
            self.do_install()
