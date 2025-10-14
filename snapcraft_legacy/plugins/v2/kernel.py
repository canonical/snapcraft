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

from typing import Any, Dict, List, Set

from overrides import overrides

from snapcraft_legacy.plugins.v2 import PluginV2
from snapcraft_legacy.project._project_options import ProjectOptions

_KERNEL_ARCH_FROM_SNAP_ARCH = {
    "i386": "x86",
    "amd64": "x86",
    "armhf": "arm",
    "arm64": "arm64",
    "ppc64el": "powerpc",
    "riscv64": "riscv",
    "s390x": "s390",
}


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

    @staticmethod
    def _get_architecture() -> str:
        target_arch = ProjectOptions().arch_build_for
        kernel_arch = _KERNEL_ARCH_FROM_SNAP_ARCH[target_arch]

        return kernel_arch

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
            "cpio",
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
            _host_arch = ProjectOptions().arch_build_on
            _target_arch = ProjectOptions().arch_build_for

            build_packages |= {
                "autoconf",
                "automake",
                "libblkid-dev",
                "libtool",
                "python3",
            }

            if _host_arch != _target_arch:
                build_packages |= {f"libc6-dev:{_target_arch}"}

        return build_packages

    @overrides
    def get_build_environment(self) -> Dict[str, str]:
        _kernel_arch = self._get_architecture()

        _kernel_image = "Image"
        _kernel_target = "modules"

        if _kernel_arch != "x86":
            _kernel_target = "modules dtbs"

        match _kernel_arch:
            case "x86" | "s390":
                _kernel_image = "bzImage"
            case "arm":
                _kernel_image = "zImage"
            case "powerpc":
                _kernel_image = "vmlinux.strip"

        return {
            "CROSS_COMPILE": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "ARCH": _kernel_arch,
            "KERNEL_IMAGE": _kernel_image,
            "KERNEL_TARGET": _kernel_target,
        }

    @overrides
    def get_build_commands(self) -> List[str]:
        kconfigflavour = self.options.kernel_kconfigflavour
        if self.options.kernel_kdefconfig != ["defconfig"]:
            kconfigflavour = ""

        return [
            " ".join(
                [
                    "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/kernel_build.sh",
                    f"kernel-kconfigflavour={kconfigflavour}",
                    f"kernel-kdefconfig={','.join(self.options.kernel_kdefconfig)}",
                    f"kernel-kconfigs={','.join(self.options.kernel_kconfigs)}",
                    f"kernel-enable-zfs={self.options.kernel_enable_zfs_support}",
                    f"kernel-enable-perf={self.options.kernel_enable_perf}",
                ]
            )
        ]
