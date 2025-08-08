# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
# pylint: disable=line-too-long,too-many-lines,attribute-defined-outside-init
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

"""The initrd plugin for building kernel snaps.

The following initramfs-specific options are provided by this plugin:
    - initrd-build-efi-image
      Optional, true if we want to create an EFI image.
      Default: false

    - initrd-modules:
      (array of string; default: none)
      list of modules to include in initrd.
      Note that kernel snaps do not provide the core boot logic which
      comes from the Ubuntu Core base snap. Include all modules you need
      for mounting the rootfs here. If installed module(s) have any
      dependencies, they are automatically installed.

    - initrd-configured-modules:
      (array of string; default: none)
      list of modules to be added to the initrd
      /lib/modules-load.d/ubuntu-core-initramfs.conf config
      to be automatically loaded.
      Configured modules are automatically added to initrd-modules.
      If module in question is not supported by the kernel, it is ignored.

    - initrd-stage-firmware:
      (boolean; default: False)
      When building initrd, required firmware is automatically added based
      on the included kernel modules. By default required firmware is searched
      in the install directory of the current part. This flag allows use of
      firmware from the stage directory instead.

    - initrd-firmware:
      (array of string; default: none)
      list of firmware files to be included in the initrd; these need to be
      relative paths to stage directory.
      <stage/part install dir>/firmware/* -> initrd:/lib/firmware/*

    - initrd-compression:
      (string; default: as defined in ubuntu-core-initrd(zstd)
      initrd compression to use; the only supported values now are
      'lz4', 'xz', 'gz', 'zstd'.

    - initrd-compression-options:
      Optional list of parameters to be passed to compressor used for initrd
      (array of string): defaults are
        gz:  -7
        lz4: -9 -l
        xz:  -7
        zstd: -1 -T0

    - initrd-overlay
      (string; default: none)
      Optional overlay to be applied to built initrd.
      This option is designed to provide easy way to apply initrd overlay for
      cases modifies initrd scripts for pre uc20 initrds.
      Value is relative path, in stage directory. and related part needs to be
      built before initrd part. During build it will be expanded to
      ${CRAFT_STAGE}/{initrd-overlay}
      Default: none

    - initrd-addons
      (array of string; default: none)
      Optional list of files to be added to the initrd.
      Function is similar to initrd-overlay, only it works on per file
      selection without a need to have overlay in dedicated directory.
      This option is designed to provide easy way to add additional content
      to initrd for cases like full disk encryption support, when device
      specific hook needs to be added to the initrd.
      Values are relative path from stage directory, so related part(s)
      need to be built before kernel part.
      During build it will be expanded to ${CRAFT_STAGE}/{initrd-addon}
      Default: none

    - initrd-add-ppa
      (boolean; default: True)
      controls if the snappy-dev PPA should be added to the system

This plugin support cross compilation, for which plugin expects
the build-environment is configured accordingly and has foreign architectures
setup accordingly.
"""

import logging
import os
from typing import Literal, cast

import pydantic
from craft_parts import infos, plugins
from overrides import overrides
from typing_extensions import Self

from snapcraft_legacy.plugins.v2 import _initrd_build, _kernel_build

logger = logging.getLogger(__name__)


class InitrdPluginProperties(plugins.PluginProperties, frozen=True):
    """The part properties used by the Initrd plugin."""

    plugin: Literal["initrd"] = "initrd"

    initrd_build_efi_image: bool = False
    initrd_modules: list[str] | None = None
    initrd_configured_modules: list[str] | None = None
    initrd_stage_firmware: bool = False
    initrd_firmware: list[str] | None = None
    initrd_compression: str | None = None
    initrd_compression_options: list[str] | None = None
    initrd_overlay: str | None = None
    initrd_addons: list[str] | None = None
    initrd_add_ppa: bool = True

    # part properties required by the plugin
    @pydantic.model_validator(mode="after")
    def validate_plugin_options(self) -> Self:
        """Validate use of initrd-compression-options."""
        # If initrd-compression-options is defined, so has to be initrd-compression.
        if self.initrd_compression_options and not self.initrd_compression:
            raise ValueError(
                "initrd-compression-options requires also initrd-compression to be defined."
            )
        return self


class InitrdPlugin(plugins.Plugin):
    """Plugin for the initrd snap build."""

    properties_class = InitrdPluginProperties

    def __init__(
        self, *, properties: plugins.PluginProperties, part_info: infos.PartInfo
    ) -> None:
        super().__init__(properties=properties, part_info=part_info)
        self.options = cast(InitrdPluginProperties, self._options)
        self._target_arch = self._part_info.target_arch
        target_arch = self._part_info.target_arch
        self._deb_arch = _kernel_build.get_deb_architecture(target_arch)

    @overrides
    def get_build_snaps(self) -> Set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> Set[str]:
        build_packages = {
            "bc",
            "binutils",
            "fakeroot",
            "dracut-core",
            "kmod",
            "kpartx",
            "systemd",
        }

        # install correct initramfs compression tool
        if self.options.initrd_compression == "lz4":
            build_packages |= {"lz4"}
        elif self.options.initrd_compression == "xz":
            build_packages |= {"xz-utils"}
        elif (
            not self.options.initrd_compression
            or self.options.initrd_compression == "zstd"
        ):
            build_packages |= {"zstd"}

        if self.options.initrd_build_efi_image:
            build_packages |= {"llvm"}

        # add snappy ppa to get correct initrd tools
        if self.options.initrd_add_ppa:
            _initrd_build.add_snappy_ppa(with_sudo=False)

        return build_packages

    @overrides
    def get_build_environment(self) -> Dict[str, str]:
        return {
            "UC_INITRD_DEB": "${CRAFT_PART_BUILD}/ubuntu-core-initramfs",
        }

    @overrides
    def get_build_commands(self) -> List[str]:
        logger.info("Getting build commands...")
        return _initrd_build.get_build_commands(
            target_arch=self._target_arch,
            initrd_modules=self.options.initrd_modules,
            initrd_configured_modules=self.options.initrd_configured_modules,
            initrd_compression=self.options.initrd_compression,
            initrd_compression_options=self.options.initrd_compression_options,
            initrd_firmware=self.options.initrd_firmware,
            initrd_addons=self.options.initrd_addons,
            initrd_overlay=self.options.initrd_overlay,
            initrd_stage_firmware=self.options.initrd_stage_firmware,
            build_efi_image=self.options.initrd_build_efi_image,
            initrd_ko_use_workaround=False,
            initrd_default_compression="zstd -1 -T0",
            initrd_include_extra_modules_conf=True,
            initrd_tool_pass_root=False,
            source_dir="${CRAFT_PART_SRC}",
            install_dir="${CRAFT_PART_INSTALL}",
            stage_dir="${CRAFT_STAGE}",
        )

    @classmethod
    def get_out_of_source_build(cls) -> bool:
        """Return whether the plugin performs out-of-source-tree builds."""
        return True
