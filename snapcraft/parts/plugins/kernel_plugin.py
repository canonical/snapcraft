# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2020-2022,2024 Canonical Ltd.
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
      (list of strings, default: defconfig))
      defconfig target to use as the base configuration. default: "defconfig"

    - kernel-kconfigflavour:
      (string; default: generic)
      Ubuntu config flavour to use as base configuration. If provided this
      option wins over kernel-kdefconfig. default: None

    - kernel-kconfigs:
      (list of strings; default: none)
      explicit list of configs to force; this will override the configs that
      were set as base through kernel-kdefconfig;
      dependent configs will be fixed using the defaults encoded in the kbuild
      config definitions.  If you don't want default for one or more implicit
      configs coming out of these, just add them to this list as well.

    - kernel-enable-zfs-support
      (boolean; default: False)
      use this flag to build in zfs support through extra ko modules

    - kernel-enable-perf
      (boolean; default: False)
      use this flag to build the perf binary

This plugin supports cross compilation, for which plugin expects
the build-environment is configured accordingly and has foreign
architectures set up accordingly.
"""

from typing import Literal, cast

from craft_parts import infos, plugins
from overrides import overrides

_KERNEL_ARCH_FROM_SNAP_ARCH = {
    "i386": "x86",
    "amd64": "x86",
    "armhf": "arm",
    "arm64": "arm64",
    "ppc64el": "powerpc",
    "riscv64": "riscv",
    "s390x": "s390",
}


class KernelPluginProperties(plugins.PluginProperties, frozen=True):
    """The part properties used by the Kernel plugin."""

    plugin: Literal["kernel"] = "kernel"

    source: str  # type: ignore[reportGeneralTypeIssues]
    kernel_kconfigs: list[str] = []
    kernel_kconfigflavour: str = "generic"
    kernel_kdefconfig: list[str] = ["defconfig"]
    kernel_enable_zfs_support: bool = False
    kernel_enable_perf: bool = False


class KernelPlugin(plugins.Plugin):
    """Plugin class implementing kernel build functionality."""

    properties_class = KernelPluginProperties

    def __init__(
        self, *, properties: plugins.PluginProperties, part_info: infos.PartInfo
    ) -> None:
        super().__init__(properties=properties, part_info=part_info)
        self.options = cast(KernelPluginProperties, self._options)

    @overrides
    def get_build_snaps(self) -> set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> set[str]:
        _base = self._part_info.base
        _host_arch = self._part_info.host_arch
        _target_arch = self._part_info.target_arch
        _zfs_enabled = self.options.kernel_enable_zfs_support

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
            "gawk",
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

        # Rust was introduced in 23.04
        if _base != "core22":
            build_packages |= {
                "clang",
                "rustc",
                "libdw-dev",
                "llvm",
            }

        if _zfs_enabled:
            build_packages |= {
                "autoconf",
                "automake",
                "libblkid-dev",
                "libtool",
                "python3",
            }

        # for cross build of zfs we also need libc6-dev:<target arch>
        if _zfs_enabled and _host_arch != _target_arch:
            build_packages |= {f"libc6-dev:{_target_arch}"}

        return build_packages

    @overrides
    def get_build_environment(self) -> dict[str, str]:
        _kernel_arch = _KERNEL_ARCH_FROM_SNAP_ARCH[self._part_info.target_arch]

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
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "ARCH": _kernel_arch,
            "KERNEL_IMAGE": _kernel_image,
            "KERNEL_TARGET": _kernel_target,
        }

    @overrides
    def get_build_commands(self) -> list[str]:
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
