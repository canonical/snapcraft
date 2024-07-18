# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-

#
# Copyright 2020-2022 Canonical Ltd.
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

"""The kernel plugin for building kernel snaps.

The following kernel-specific options are provided by this plugin:

    - kernel-kdefconfig:
      (list of kdefconfigs, default: none))
      defconfig target to use as the base configuration. default: "defconfig"

    - kernel-kconfigflavour:
      (string; default: generic)
      Ubuntu config flavour to use as base configuration. If provided this
      option wins over kernel-kdefconfig. default: None

    - kernel-kconfigs:
      (list of strings; default: none)
      explicit list of configs to force; this will override the configs that
      were set as base through kernel-kdefconfig and dependent configs
      will be fixed using the defaults encoded in the kbuild config
      definitions.  If you don't want default for one or more implicit configs
      coming out of these, just add them to this list as well.

    - kernel-image-target:
      (yaml object, string or null for default target)
      the default target is bzImage and can be set to any specific
      target.
      For more complex cases where one would want to use
      the same snapcraft.yaml to target multiple architectures a
      yaml object can be used. This yaml object would be a map of
      debian architecture and kernel image build targets.

    - kernel-enable-zfs-support
      (boolean; default: False)
      use this flag to build in zfs support through extra ko modules

    - kernel-enable-perf
      (boolean; default: False)
      use this flag to build the perf binary

This plugin support cross compilation, for which plugin expects
the build-environment is configured accordingly and has foreign architectures
setup accordingly.
"""

import logging
import os
import sys
from typing import Any, Dict, List, Set

from overrides import overrides

from snapcraft_legacy.plugins.v2 import PluginV2
from snapcraft_legacy.project._project_options import ProjectOptions

from . import _kernel_build

logger = logging.getLogger(__name__)

_default_kernel_image_target = {
    "amd64": "bzImage",
    "armhf": "zImage",
    "arm64": "Image",
    "powerpc": "uImage",
    "ppc64el": "vmlinux.strip",
    "s390x": "bzImage",
    "riscv64": "Image",
}


class KernelPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "kernel-kdefconfig": {"type": "array", "default": []},
                "kernel-kconfigflavour": {"type": "string", "default": "defconfig"},
                "kernel-kconfigs": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "kernel-image-target": {
                    "oneOf": [{"type": "string"}, {"type": "object"}],
                    "default": "",
                },
                "kernel-enable-zfs-support": {
                    "type": "boolean",
                    "default": False,
                },
                "kernel-enable-perf": {
                    "type": "boolean",
                    "default": False,
                },
            },
        }

    def __init__(self, *, part_name: str, options) -> None:
        super().__init__(part_name=str, options=options)
        self.name = part_name
        self.options = options

        target_arch = _get_target_architecture()
        self._deb_arch = _kernel_build.get_deb_architecture(target_arch)
        self._kernel_arch = _kernel_build.get_kernel_architecture(target_arch)
        self._target_arch = target_arch

        # check if we are cross building
        host_arch = os.getenv("SNAP_ARCH")
        self._cross_building = False
        if host_arch != self._target_arch:
            self._cross_building = True

    def _init_build_env(self) -> None:
        # first get all the architectures, new v2 plugin is making life difficult
        logger.info("Initializing build env...")

        self._make_cmd = ["make", "-j$(nproc)"]
        # we are building out of tree, configure paths
        self._make_cmd.append("-C")
        self._make_cmd.append("${KERNEL_SRC}")
        self._make_cmd.append("O=${SNAPCRAFT_PART_BUILD}")

        self._check_cross_compilation()
        self._set_kernel_targets()

    def _check_cross_compilation(self) -> None:
        if self._cross_building:
            self._make_cmd.append(f"ARCH={self._kernel_arch}")
            self._make_cmd.append("CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-")

    def _set_kernel_targets(self) -> None:
        if not self.options.kernel_image_target:
            self.kernel_image_target = _default_kernel_image_target[self._deb_arch]
        elif isinstance(self.options.kernel_image_target, str):
            self.kernel_image_target = self.options.kernel_image_target
        elif self._deb_arch in self.options.kernel_image_target:
            self.kernel_image_target = self.options.kernel_image_target[self._deb_arch]

        self._make_targets = [self.kernel_image_target, "modules"]
        self._make_install_targets = [
            "modules_install",
            "INSTALL_MOD_STRIP=1",
            "INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
        ]
        if self._kernel_arch in ("arm", "arm64", "riscv64"):
            self._make_targets.append("dtbs")
            self._make_install_targets.extend(
                ["dtbs_install", "INSTALL_DTBS_PATH=${SNAPCRAFT_PART_INSTALL}/dtbs"]
            )

    @overrides
    def get_build_snaps(self) -> Set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> Set[str]:
        build_packages = {
            "bc",
            "binutils",
            "bison",
            "cmake",
            "cryptsetup",
            "debhelper",
            "fakeroot",
            "flex",
            "gcc",
            "kmod",
            "kpartx",
            "libelf-dev",
            "libssl-dev",
            "systemd",
        }

        if self.options.kernel_enable_zfs_support:
            build_packages |= {
                "autoconf",
                "automake",
                "libblkid-dev",
                "libtool",
                "python3",
            }

        # for cross build of zfs we also need libc6-dev:<target arch>
        if self.options.kernel_enable_zfs_support and self._cross_building:
            build_packages |= {f"libc6-dev:{self._target_arch}"}

        return build_packages

    @overrides
    def get_build_environment(self) -> Dict[str, str]:
        logger.info("Getting build env...")
        self._init_build_env()

        return {
            "CROSS_COMPILE": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "ARCH": self._kernel_arch,
            "DEB_ARCH": "${SNAPCRAFT_TARGET_ARCH}",
            "KERNEL_BUILD_ARCH_DIR": f"${{SNAPCRAFT_PART_BUILD}}/arch/{self._kernel_arch}/boot",
            "KERNEL_IMAGE_TARGET": self.kernel_image_target,
        }

    @overrides
    def get_build_commands(self) -> List[str]:
        logger.info("Getting build commands...")
        return _kernel_build.get_build_commands(
            make_cmd=self._make_cmd.copy(),
            make_targets=self._make_targets,
            make_install_targets=self._make_install_targets,
            target_arch=self._target_arch,
            target_arch_triplet="${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}",
            config_flavour=self.options.kernel_kconfigflavour,
            defconfig=self.options.kernel_kdefconfig,
            configs=self.options.kernel_kconfigs,
            enable_zfs_support=self.options.kernel_enable_zfs_support,
            enable_perf=self.options.kernel_enable_perf,
            project_dir="${SNAPCRAFT_PROJECT_DIR}",
            source_dir="${SNAPCRAFT_PART_SRC}",
            build_dir="${SNAPCRAFT_PART_BUILD}",
            install_dir="${SNAPCRAFT_PART_INSTALL}",
        )

    @property
    def out_of_source_build(self):
        """Return whether the plugin performs out-of-source-tree builds."""
        return True


def _get_target_architecture() -> str:
    # TODO: there is bug in snapcraft and target arch is not
    # reported correctly.
    # As work around check if we are cross building, to know what is
    # target arch
    target_arch = None
    for i in range(1, len(sys.argv)):
        if sys.argv[i].startswith("--target-arch="):
            target_arch = sys.argv[i].split("=")[1]
        elif sys.argv[i].startswith("--target-arch"):
            target_arch = sys.argv[i + 1]

    if target_arch is None:
        # this is native build, use deb_arch
        # as for native build it's reported correctly
        target_arch = ProjectOptions().deb_arch

    logger.info(f"Target architecture: {target_arch}")
    return target_arch
