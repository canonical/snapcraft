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
      (list of kdefconfigs)
      defconfig target to use as the base configuration. default: "defconfig"

    - kernel-kconfigfile:
      (filepath; default: none)
      path to file to use as base configuration; if provided, this option wins
      over everything else. default: None

    - kernel-kconfigflavour:
      (string; default: none)
      Ubuntu config flavour to use as base configuration. If provided this
      option wins over kernel-kdefconfig. default: None

    - kernel-kconfigs:
      (list of strings; default: none)
      explicit list of configs to force; this will override the configs that
      were set as base through kernel-kdefconfig and kernel-kconfigfile;
      dependent configs will be fixed using the defaults encoded in the kbuild
      config definitions.  If you don't want default for one or more implicit
      configs coming out of these, just add them to this list as well.

    - kernel-image-target:
      (yaml object, string or null for default target)
      the default target is bzImage and can be set to any specific
      target.
      For more complex cases where one would want to use
      the same snapcraft.yaml to target multiple architectures a
      yaml object can be used. This yaml object would be a map of
      debian architecture and kernel image build targets.

    - kernel-with-firmware:
      (boolean; default: True)
      use this flag to disable shipping binary firmwares.

    - kernel-device-trees:
      (array of string; default: none)
      list of device trees to build, the format is <device-tree-name>.dts.

    - kernel-compiler
      (string; default: none)
      Optional, define compiler to use, by default gcc compiler is used.
      Other permitted compilers: clang

    - kernel-compiler-parameters
      (array of string)
      Optional, define extra compiler parameters to be passed to the compiler.
      Default value is empty.

    - kernel-enable-zfs-support
      (boolean; default: False)
      use this flag to build in zfs support through extra ko modules

    - kernel-enable-perf
      (boolean; default: False)
      use this flag to build the perf binary

    - kernel-use-llvm
      (boolean or string to specify version suffix; default: False)
      Use the LLVM substitutes for the GNU binutils utilities. Set this to a
      string (e.g. "-12") to use a specific version of the LLVM utilities.

This plugin supports cross compilation, for which plugin expects
the build-environment is configured accordingly and has foreign
architectures set up accordingly.
"""

import logging
import re
from typing import Any, Literal, cast

import pydantic
from craft_parts import infos, plugins
from overrides import overrides
from typing_extensions import Self

from snapcraft_legacy.plugins.v2 import _kernel_build

logger = logging.getLogger(__name__)

_default_kernel_image_target = {
    "amd64": "bzImage",
    "i386": "bzImage",
    "armhf": "zImage",
    "arm64": "Image.gz",
    "powerpc": "uImage",
    "ppc64el": "vmlinux.strip",
    "s390x": "bzImage",
    "riscv64": "Image",
}


class KernelPluginProperties(plugins.PluginProperties, frozen=True):
    """The part properties used by the Kernel plugin."""

    plugin: Literal["kernel"] = "kernel"

    kernel_kdefconfig: list[str] = ["defconfig"]
    kernel_kconfigfile: str | None = None
    kernel_kconfigflavour: str | None = None
    kernel_kconfigs: list[str] | None = None
    kernel_image_target: str | dict[str, Any] | None = None
    kernel_with_firmware: bool = True
    kernel_device_trees: list[str] | None = None
    kernel_compiler: str | None = None
    kernel_compiler_parameters: list[str] | None = None
    kernel_enable_zfs_support: bool = False
    kernel_enable_perf: bool = False
    kernel_use_llvm: bool | str = False

    # part properties required by the plugin
    @pydantic.model_validator(mode="after")
    def validate_plugin_options(self) -> Self:
        """If kernel-image-target is defined, it has to be string or dictionary."""
        if self.kernel_image_target:
            if not isinstance(self.kernel_image_target, str):
                if not isinstance(self.kernel_image_target, dict):
                    raise ValueError(
                        f"kernel-image-target is in invalid format(type{type(self.kernel_image_target)}). It should be either string or dictionary."
                    )
        return self


class KernelPlugin(plugins.Plugin):
    """Plugin for the kernel snap build."""

    properties_class = KernelPluginProperties

    def __init__(
        self, *, properties: plugins.PluginProperties, part_info: infos.PartInfo
    ) -> None:
        super().__init__(properties=properties, part_info=part_info)
        self.options = cast(KernelPluginProperties, self._options)

        target_arch = self._part_info.target_arch
        self._deb_arch = _kernel_build.get_deb_architecture(target_arch)
        self._kernel_arch = _kernel_build.get_kernel_architecture(target_arch)
        self._target_arch = target_arch

        # check if we are cross building
        self._cross_building = False
        if (
            self._part_info.project_info.host_arch
            != self._part_info.project_info.target_arch
        ):
            self._cross_building = True
        self._llvm_version = self._determine_llvm_version()

    def _determine_llvm_version(self) -> str | None:
        if (
            isinstance(self.options.kernel_use_llvm, bool)
            and self.options.kernel_use_llvm
        ):
            return "1"
        if isinstance(self.options.kernel_use_llvm, str):
            suffix = re.match(r"^-\d+$", self.options.kernel_use_llvm)
            if suffix is None:
                raise ValueError(
                    f'kernel-use-llvm must match the format "-<version>" (e.g. "-12"), not "{self.options.kernel_use_llvm}"'
                )
            return self.options.kernel_use_llvm
        # Not use LLVM utilities
        return None

    def _init_build_env(self) -> None:
        # first get all the architectures, new v2 plugin is making life difficult
        logger.info("Initializing build env...")

        self._make_cmd = ["make", "-j$(nproc)"]
        # we are building out of tree, configure paths
        self._make_cmd.append("-C")
        self._make_cmd.append("${KERNEL_SRC}")
        self._make_cmd.append("O=${CRAFT_PART_BUILD}")

        self._check_cross_compilation()
        self._set_kernel_targets()
        self._set_llvm()

    def _check_cross_compilation(self) -> None:
        if self._cross_building:
            self._make_cmd.append(f"ARCH={self._kernel_arch}")
            self._make_cmd.append("CROSS_COMPILE=${CRAFT_ARCH_TRIPLET_BUILD_FOR}-")

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
            "INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
        ]
        if self.options.kernel_device_trees:
            self.dtbs = [f"{i}.dtb" for i in self.options.kernel_device_trees]
            if self.dtbs:
                self._make_targets.extend(self.dtbs)
        elif self._kernel_arch in ("arm", "arm64", "riscv", "riscv64"):
            self._make_targets.append("dtbs")
            self._make_install_targets.extend(
                ["dtbs_install", "INSTALL_DTBS_PATH=${CRAFT_PART_INSTALL}/dtbs"]
            )
        self._make_install_targets.extend(self._get_fw_install_targets())

    def _set_llvm(self) -> None:
        if self._llvm_version is not None:
            self._make_cmd.append(f'LLVM="{self._llvm_version}"')

    def _get_fw_install_targets(self) -> list[str]:
        if not self.options.kernel_with_firmware:
            return []

        return [
            "firmware_install",
            "INSTALL_FW_PATH=${CRAFT_PART_INSTALL}/lib/firmware",
        ]

    def _configure_compiler(self) -> None:
        # check if we are using gcc or another compiler
        if self.options.kernel_compiler:
            # at the moment only clang is supported as alternative, warn otherwise
            kernel_compiler = re.match(r"^clang(-\d+)?$", self.options.kernel_compiler)
            if kernel_compiler is None:
                logger.warning("Only other 'supported' compiler is clang")
                logger.info("hopefully you know what you are doing")
            self._make_cmd.append(f'CC="{self.options.kernel_compiler}"')
        if self.options.kernel_compiler_parameters:
            for opt in self.options.kernel_compiler_parameters:
                self._make_cmd.append(str(opt))

    @overrides
    def get_build_snaps(self) -> set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> set[str]:
        build_packages = {
            "bc",
            "binutils",
            "debhelper",
            "fakeroot",
            "gcc",
            "cmake",
            "cryptsetup",
            "kmod",
            "kpartx",
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

        if self._llvm_version is not None:
            # Use the specified version suffix for the packages if it has been
            # set by the user
            suffix = self._llvm_version if self._llvm_version != "1" else ""
            llvm_packages = [
                "llvm",
                "lld",
            ]
            build_packages |= {f"{f}{suffix}" for f in llvm_packages}

        return build_packages

    @overrides
    def get_build_environment(self) -> dict[str, str]:
        logger.info("Getting build env...")
        self._init_build_env()

        return {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "ARCH": self._kernel_arch,
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
            "KERNEL_BUILD_ARCH_DIR": f"${{CRAFT_PART_BUILD}}/arch/{self._kernel_arch}/boot",
            "KERNEL_IMAGE_TARGET": self.kernel_image_target,
        }

    @overrides
    def get_build_commands(self) -> list[str]:
        logger.info("Getting build commands...")
        self._configure_compiler()
        return _kernel_build.get_build_commands(
            make_cmd=self._make_cmd.copy(),
            make_targets=self._make_targets,
            make_install_targets=self._make_install_targets,
            target_arch=self._target_arch,
            target_arch_triplet="${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
            config_file=self.options.kernel_kconfigfile,
            config_flavour=self.options.kernel_kconfigflavour,
            defconfig=self.options.kernel_kdefconfig,
            configs=self.options.kernel_kconfigs,
            device_trees=self.options.kernel_device_trees,
            enable_zfs_support=self.options.kernel_enable_zfs_support,
            enable_perf=self.options.kernel_enable_perf,
            project_dir="${CRAFT_PROJECT_DIR}",
            source_dir="${CRAFT_PART_SRC}",
            build_dir="${CRAFT_PART_BUILD}",
            install_dir="${CRAFT_PART_INSTALL}",
            stage_dir="${CRAFT_STAGE}",
        )

    @classmethod
    def get_out_of_source_build(cls) -> bool:
        """Return whether the plugin performs out-of-source-tree builds."""
        return True
