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

from snapcraft_legacy.plugins.v2.kernel import KernelPlugin

def test_schema():
    assert KernelPlugin.get_schema() == {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "kernel-kdefconfig": {
                "type": "array",
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

def test_get_build_packages():
    plugin = KernelPlugin(part_name="my-part", options=lambda: None)

    assert plugini.get_build_packages() == {
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

def test_get_build_packages_zfs():
    class Options:
        kernel_enable_zfs_support = True

    plugin = KernelPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_packages() == {
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
        "autoconf",
        "automake",
        "libblkid-dev",
        "libtool",
        "python3",
    }

def test_get_build_environment():
    plugin = KernelPlugin(part_name="my-part", options=lambda: None)

    assert plugin.get_build_environment() == {
        "CRAFT_ARCH_TRIPLET_BUILD_FOR": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}",
        "CRAFT_PROJECT_DIR": "${SNAPCRAFT_PROJECT_DIR}",
        "CRAFT_PART_SRC": "${SNAPCRAFT_PART_SRC}",
        "CRAFT_PART_BUILD": "${SNAPCRAFT_PART_BUILD}",
        "CRAFT_PART_INSTALL": "${SNAPCRAFT_PART_INSTALL}",
        "CRAFT_TARGET_ARCH": "${SNAPCRAFT_TARGET_ARCH}",
        "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
        "ARCH": "${CRAFT_RRCH_BUILD_FOR}",
    }

def test_get_build_commands():
    class Options:
        kernel_kdefconfig = ["snappy_defconfig", "foo_config"]
        kernel_kconfigflavour = "aflavour"
        kernel_kconfigs = ["CONFIG_FOO=y", "CONFIG_BAR=m"]
        kernel_enable_zfs_support = True
        kernel_enable_perf = True

    plugin = KernelPlugin(part_name="my-part", options=Options())

    assert plugin.get_build_commands() == {
        " ".join(
            "$SNAP/lib/python3.12/siite-packages/snapcraft/parts/plugins/kernel_build.sh",
            "flavour=aflavour",
            "defconfig=snappy_defconfig,foo_config",
            "configs=CONFIG_FOO=y,CONFIG_BAR=m",
            "enable_zfs_support=True",
            "enable_perf=True",
        )
    }
