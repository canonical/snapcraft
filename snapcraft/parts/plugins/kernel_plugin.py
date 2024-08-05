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
      (list of kdefconfigs, default: none))
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

This plugin supports cross compilation, for which plugin expects
the build-environment is configured accordingly and has foreign
architectures set up accordingly.
"""

import logging
from typing import Any, Literal, cast

import pydantic
from craft_parts import infos, plugins
from overrides import overrides
from typing_extensions import Self

from snapcraft_legacy.plugins.v2 import _kernel_build

logger = logging.getLogger(__name__)


class KernelPluginProperties(plugins.PluginProperties, frozen=True):
    """The part properties used by the Kernel plugin."""

    plugin: Literal["kernel"] = "kernel"

    kernel_kconfigs: list[str] | None = None
    kernel_kconfigflavour: str = "generic"
    kernel_kdefconfig: list[str] | None = None
    kernel_image_target: str | dict[str, Any] | None = None
    kernel_enable_zfs_support: bool = False
    kernel_enable_perf: bool = False

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
    """Plugin class implementing kernel build functionality."""

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

        # set kernel targets
        if not self.options.kernel_image_target:
            self.kernel_image_target = _kernel_build.default_kernel_image_target[
                self._deb_arch
            ]
        elif isinstance(self.options.kernel_image_target, str):
            self.kernel_image_target = self.options.kernel_image_target
        elif self._deb_arch in self.options.kernel_image_target:
            self.kernel_image_target = self.options.kernel_image_target[self._deb_arch]

    @overrides
    def get_build_snaps(self) -> set[str]:  # pylint: disable=missing-function-docstring
        return set()

    @overrides
    def get_build_packages(
        self,
    ) -> set[str]:  # pylint: disable=missing-function-docstring
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
    def get_build_environment(
        self,
    ) -> dict[str, str]:  # pylint: disable=missing-function-docstring
        logger.info("Getting build env...")
        return {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "ARCH": self._kernel_arch,
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
            "KERNEL_BUILD_ARCH_DIR": f"${{CRAFT_PART_BUILD}}/arch/{self._kernel_arch}/boot",
            "KERNEL_IMAGE_TARGET": self.kernel_image_target,
        }

    @overrides
    def get_build_commands(
        self,
    ) -> list[str]:  # pylint: disable=missing-function-docstring
        logger.info("Getting build commands...")
        kconfigflavour = self.options.kernel_kconfigflavour
        if self.options.kernel_kdefconfig:
            kconfigflavour = ""
        return _kernel_build.get_build_commands(
            kernel_arch=self._kernel_arch,
            config_flavour=kconfigflavour,
            defconfig=self.options.kernel_kdefconfig,
            configs=self.options.kernel_kconfigs,
            enable_zfs_support=self.options.kernel_enable_zfs_support,
            enable_perf=self.options.kernel_enable_perf,
        )

    @classmethod
    def get_out_of_source_build(cls) -> bool:
        """Return whether the plugin performs out-of-source-tree builds."""
        return True
