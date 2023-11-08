# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
# pylint: disable=line-too-long,too-many-lines,attribute-defined-outside-init
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

"""The initrd plugin for building kernel snaps.

The following initramfs-specific options are provided by this plugin:
    - initrd-build-efi-image
      Optional, true if we want to create an EFI image.
      Default: false

    - initrd-kernel-image-target
      Optional, and only required if initrd-build-efi-image is set
      (yaml object, string or null for default target)
      the default target is bzImage and can be set to any specific
      target.
      For more complex cases where one would want to use
      the same snapcraft.yaml to target multiple architectures a
      yaml object can be used. This yaml object would be a map of
      debian architecture and kernel image build targets.

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
the build-environment is comfigured accordingly and has foreign architectures
setup accordingly.
"""

import logging
from typing import Any, Dict, List, Optional, Set, cast

from craft_parts import infos, plugins
from overrides import overrides
from pydantic import root_validator

from snapcraft.parts.plugins import kernel_plugin
from snapcraft_legacy.plugins.v2 import _initrd_build
from snapcraft_legacy.plugins.v2 import _kernel_build

logger = logging.getLogger(__name__)

class InitrdPluginProperties(plugins.PluginProperties, plugins.PluginModel):
    """The part properties used by the Initrd plugin."""

    initrd_build_efi_image: bool = False
    initrd_kernel_image_target: Any
    initrd_modules: Optional[List[str]]
    initrd_configured_modules: Optional[List[str]]
    initrd_stage_firmware: bool = False
    initrd_firmware: Optional[List[str]]
    initrd_compression: Optional[str]
    initrd_compression_options: Optional[List[str]]
    initrd_overlay: Optional[str]
    initrd_addons: Optional[List[str]]
    initrd_add_ppa: bool = True

    # part properties required by the plugin
    @root_validator
    @classmethod
    def validate_pluging_options(cls, values):
        """Validate use of initrd-compression-options initrd_kernel_image_target."""
        # If initrd-compression-options is defined, so has to be initrd-compression.
        if values.get("initrd_compression_options") and not values.get(
            "initrd_compression"
        ):
            raise ValueError(
                "initrd-compression-options requires also initrd-compression to be defined."
            )

        # If initrd-kernel-image-target is defined, it has to be string or dictionary
        if values.get("initrd_kernel_image_target"):
            if not isinstance(values.get("initrd_kernel_image_target"), str):
                if not isinstance(values.get("initrd_kernel_image_target"), dict):
                    raise ValueError(
                        f'initrd_kernel-image-target is in invalid format(type{type(values.get("initrd_kernel_image_target"))}). It should be either string or dictionary.'
                    )

        return values

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]):
        """Populate class attributes from the part specification.

        :param data: A dictionary containing part properties.

        :return: The populated plugin properties data object.

        :raise pydantic.ValidationError: If validation fails.
        """
        plugin_data = plugins.extract_plugin_properties(
            data, plugin_name="initrd", required=[]
        )
        return cls(**plugin_data)


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
        if not self.options.initrd_kernel_image_target:
            self.kernel_image_target = kernel_plugin.default_kernel_image_target[self._deb_arch]
        elif isinstance(self.options.initrd_kernel_image_target, str):
            self.kernel_image_target = self.options.initrd_kernel_image_target
        elif self._deb_arch in self.options.initrd_kernel_image_target:
            self.kernel_image_target = self.options.initrd_kernel_image_target[self._deb_arch]

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
            kernel_image_target=self.kernel_image_target,
            initrd_ko_use_workaround=False,
            initrd_default_compression="zstd -1 -T0",
            initrd_include_extra_modules_conf=True,
            initrd_tool_pass_root=False,
            install_dir="${CRAFT_PART_INSTALL}",
            stage_dir="${CRAFT_STAGE}",
        )

    @classmethod
    def get_out_of_source_build(cls) -> bool:
        """Return whether the plugin performs out-of-source-tree builds."""
        return True
