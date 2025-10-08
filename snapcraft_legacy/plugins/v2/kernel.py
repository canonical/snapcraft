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
      (list of kdefconfigs, default: defconfig)
      Kernel defconfig target to use as the base configuration.

    - kernel-kconfigflavour:
      (string; default: generic)
      Ubuntu config flavour to use as base configuration. This option supercedes
      kernel-kdefconfig.

    - kernel-kconfigs:
      (list of strings; default: none)
      Explicit list of kernel config options to force; this will override the
      configs that were set as base through kernel-kdefconfig and dependent
      configs will be fixed using the defaults encoded in the Kbuild config
      definitions.

    - kernel-enable-zfs-support
      (boolean; default: false)
      Build ZFS kernel modules.

    - kernel-enable-perf
      (boolean; default: false)
      Enable or disable building the perf binary.
"""

import logging
import os
import sys
from typing import Any, Dict, List, Set

from overrides import overrides

from snapcraft_legacy.plugins.v2 import PluginV2
from snapcraft_legacy.project._project_options import ProjectOptions

logger = logging.getLogger(__name__)


class KernelPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "kernel-kdefconfig": {
                    "type": "array",
                    "items": {"type": "string"},
                    "default": ["defconfig"],
                },
                "kernel-kconfigflavour": {"type": "string", "default": "generic"},
                "kernel-kconfigs": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
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
        super().__init__(part_name=part_name, options=options)
        self.name = part_name
        self.options = options

        target_arch = _get_target_architecture()
        self._target_arch = target_arch

        # check if we are cross building
        host_arch = os.getenv("SNAP_ARCH")
        self._cross_building = False
        if host_arch != self._target_arch:
            self._cross_building = True

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
            "lz4",
            "systemd",
            "xz-utils",
            "zstd",
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

        _kernel_arch = ""
        _kernel_image = ""
        _kernel_target = ""

        if self._part_info.target_arch == "amd64":
            _kernel_arch = "x86"
            _kernel_image = "bzImage"
            _kernel_target = "modules"
        if self._part_info.target_arch == "arm64":
            _kernel_arch = "arm64"
            _kernel_image = "Image"
            _kernel_target = "modules dtbs"
        if self._part_info.target_arch == "armhf":
            _kernel_arch = "arm"
            _kernel_image = "zImage"
            _kernel_target = "modules dtbs"
        if self._part_info.target_arch == "ppc64el":
            _kernel_arch = "powerpc"
            _kernel_image = "vmlinux.strip"
            _kernel_target = "modules dtbs"
        if self._part_info.target_arch == "powerpc":
            _kernel_arch = "powerpc"
            _kernel_image = "uImage"
            _kernel_target = "modules dtbs"
        if self._part_info.target_arch == "riscv64":
            _kernel_arch = "riscv"
            _kernel_image = "Image"
            _kernel_target = "modules dtbs"
        if self._part_info.target_arch == "s390x":
            _kernel_arch = "s390"
            _kernel_image = "bzImage"
            _kernel_target = "modules dtbs"

        return {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "ARCH": _kernel_arch,
            "KERNEL_IMAGE": _kernel_image,
            "KERNEL_TARGET": _kernel_target,
        }

    @overrides
    def get_build_commands(self) -> List[str]:
        logger.info("Getting build commands...")
        kconfigflavour = self.options.kernel_kconfigflavour
        if self.options.kernel_kdefconfig != "defconfig":
            kconfigflavour = ""

        return [
            " ".join(
                [
                    "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/kernel_build.sh",
                    f"flavour={kconfigflavour}",
                    f"defconfig={' '.join(self.options.kernel_kdefconfig)}",
                    f"configs={','.join(self.options.kernel_kconfigs)}",
                    f"enable_zfs_support={self.options.kernel_enable_zfs_support}",
                    f"enable_perf={self.options.kernel_enable_perf}",
                ]
            )
        ]


def _get_target_architecture() -> str:
    # TODO: there is bug in snapcraft and target arch is not
    # reported correctly.
    # As work around check if we are cross building, to know what is
    # target arch
    target_arch = None
    # pylint: disable=invalid-name
    for i in range(1, len(sys.argv)):
        if sys.argv[i].startswith("--target-arch="):
            target_arch = sys.argv[i].split("=")[1]
        elif sys.argv[i].startswith("--target-arch"):
            target_arch = sys.argv[i + 1]

    if target_arch is None:
        # this is native build, use deb_arch
        # as for native build it's reported correctly
        target_arch = ProjectOptions().deb_arch

    logger.info("Target architecture: %s", target_arch)
    return target_arch
