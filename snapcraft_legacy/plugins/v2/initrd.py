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

    - initrd-firmware:
      (array of string; default: none)
      list of firmware files to be included in the initrd; these need to be
      relative paths to stage directory.
      <stage/part install dir>/firmware/* -> initrd:/lib/firmware/*

    - initrd-compression:
      (string; default: as defined in ubuntu-core-initrd(lz4)
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
      ${SNAPCRAFT_STAGE}/{initrd-overlay}
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
      During build it will be expanded to
      ${SNAPCRAFT_STAGE}/{initrd-addon}
      Default: none

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

from . import _initrd_build

logger = logging.getLogger(__name__)


class InitrdPlugin(PluginV2):
    """Plugin class implementing initrd build functionality"""

    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "initrd-modules": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "initrd-configured-modules": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "initrd-firmware": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "initrd-compression": {
                    "type": "string",
                    "enum": ["lz4", "xz", "gz", "zstd"],
                },
                "initrd-compression-options": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "initrd-overlay": {
                    "type": "string",
                    "default": "",
                },
                "initrd-addons": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
        }

    def __init__(self, *, part_name: str, options) -> None:
        super().__init__(part_name=part_name, options=options)
        self.name = part_name
        self.options = options
        self._target_arch = _get_target_architecture()
        # check if we are cross building
        self._host_arch = os.getenv("SNAP_ARCH", "")
        self._cross_building = False
        if self._host_arch != self._target_arch:
            self._cross_building = True
        self._ubuntu_series = "focal"

    @overrides
    def get_build_snaps(self) -> Set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> Set[str]:
        build_packages = {
            "curl",
            "fakechroot",
            "fakeroot",
        }
        # consider cross-build option
        if self._cross_building:
            build_packages |= {
                f"libfakechroot:{self._target_arch}",
                f"libfakeroot:{self._target_arch}",
            }
        return build_packages

    @overrides
    def get_build_environment(self) -> Dict[str, str]:
        return {
            "UC_INITRD_ROOT_NAME": "uc-initramfs-build-root",
            "UC_INITRD_ROOT": "${SNAPCRAFT_PART_SRC}/${UC_INITRD_ROOT_NAME}",
            "KERNEL_MODULES": "${SNAPCRAFT_STAGE}/modules",
            "KERNEL_FIRMWARE": "${SNAPCRAFT_STAGE}/firmware",
            "UBUNTU_SERIES": self._ubuntu_series,
            "UBUNTU_CORE_BASE": "core20",
            "CRAFT_ARCH_TRIPLET_BUILD_FOR": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}",
            "CRAFT_ARCH_BUILD_FOR": self._target_arch,
            "CRAFT_ARCH_BUILD_ON": self._host_arch,
            "CRAFT_STAGE": "${SNAPCRAFT_STAGE}",
            "CRAFT_PART_SRC": "${SNAPCRAFT_PART_SRC}",
            "CRAFT_PART_BUILD": "${SNAPCRAFT_PART_BUILD}",
            "CRAFT_PART_INSTALL": "${SNAPCRAFT_PART_INSTALL}",
        }

    @overrides
    def get_build_commands(self) -> List[str]:
        logger.info("Getting build commands...")
        return _initrd_build.get_build_commands(
            initrd_modules=self.options.initrd_modules,
            initrd_configured_modules=self.options.initrd_configured_modules,
            initrd_compression=self.options.initrd_compression,
            initrd_compression_options=self.options.initrd_compression_options,
            initrd_firmware=self.options.initrd_firmware,
            initrd_addons=self.options.initrd_addons,
            initrd_overlay=self.options.initrd_overlay,
            initrd_ko_use_workaround=True,
            initrd_default_compression="lz4 -9 -l",
        )

    @property
    def out_of_source_build(self):
        """Return whether the plugin performs out-of-source-tree builds."""
        return True


def _get_target_architecture() -> str:
    # TODO: there is bug in snapcraft and target arch is not reported
    # correctly. As work around check if we are cross building,
    # to know what is target arch
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

    return target_arch
