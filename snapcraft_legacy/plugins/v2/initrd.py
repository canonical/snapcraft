# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
# pylint: disable=line-too-long,too-many-lines,attribute-defined-outside-init
#
# Copyright 2025 Canonical Ltd.
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

This plugin uses the following plugin-specific keywords:

    - initrd-addons
      (list of strings; default: none)
      A list of files to include in the initrd. Provided as relative paths to
      the stage directory.

    - initrd-firmware:
      (list of strings; default: none)
      A list of firmware to include in the initrd. Provided as relative paths to
      the stage directory.

    - initrd-modules:
      (list of strings; default: none)
      A list of module name to include in the initrd. If the specified
      module(s) have any dependencies, they are also installed.
"""

import logging
from typing import Any, Dict, List, Set

from overrides import overrides

from snapcraft_legacy.plugins.v2 import PluginV2

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
                "initrd-firmware": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
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

    @overrides
    def get_build_snaps(self) -> Set[str]:
        base = self._part_info.base

        build_snaps = {
            base,
        }
        return build_snaps

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

    @property
    def out_of_source_build(self):
        """Return whether the plugin performs out-of-source-tree builds."""
        return True

    @overrides
    def get_build_commands(self) -> List[str]:
        logger.info("Getting build commands...")
        return [
            " ".join(
                [
                    "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/initrd_build.sh",
                    f"initrd-modules={self.options.initrd_modules}",
                    f"initrd-firmware={self.options.initrd_firmware}",
                    f"initrd-addons={self.options.initrd_addons}",
                ]
            )
        ]
