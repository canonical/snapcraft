# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2022 Canonical Ltd
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

from snapcraft_legacy.plugins.v2.initrd import InitrdPlugin

def test_schema():
    assert InitrdPlugin.get_schema() == {
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

def test_get_build_packages():
    plugin = InitrdPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_packages() == {
        "curl",
        "fakechroot",
        "fakeroot",
    }

def test_get_build_snaps():
    plugin = InitrdPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_snaps() == {
        "core20"
    }

def test_get_build_environment():
    plugin = InitrdPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_environment() == {
        "UC_INITRD_ROOT_NAME": "uc-initramfs-build-root",
        "UC_INITRD_ROOT": "${SNAPCRAFT_PART_SRC}/${UC_INITRD_ROOT_NAME}",
        "KERNEL_MODULES": "${SNAPCRAFT_STAGE}/modules",
        "KERNEL_FIRMWARE": "${SNAPCRAFT_STAGE}/firmware",
        "UBUNTU_SERIES": "focal",
        "UBUNTU_CORE_BASE": "core20",
        "CRAFT_ARCH_TRIPLET_BUILD_FOR": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}",
        "CRAFT_ARCH_BUILD_FOR": "${SNAPCRAFT_ARCH_BUILD_FOR}",
        "CRAFT_ARCH_BUILD_ON": "${SNAPCRAFT_ARCH_BUILD_ON}",
        "CRAFT_STAGE": "${SNAPCRAFT_STAGE}",
        "CRAFT_PART_SRC": "${SNAPCRAFT_PART_SRC}",
        "CRAFT_PART_BUILD": "${SNAPCRAFT_PART_BUILD}",
        "CRAFT_PART_INSTALL": "${SNAPCRAFT_PART_INSTALL}",
    }

def test_get_build_commands():
    class Options:
        initrd_addons = "usr/bin/foo"
        initrd_firmware = "bar.bin"
        initrd_modules = "baz"

    plugin = InitrdPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == {
        " ".join(
            "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugin/initrd_build.sh",
            "initrd-modules=baz",
            "initrd-firmware=bar.bin"
            "initrd-addons=usr/bin/foo",
        )
    }
