# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import pytest
from craft_parts import Part, PartInfo, ProjectInfo

from snapcraft.parts.plugins import KernelPlugin


@pytest.fixture(autouse=True)
def part_info(new_dir):
    yield PartInfo(
        project_info=ProjectInfo(
            application_name="test", project_name="test-snap", cache_dir=new_dir
        ),
        part=Part("my-part", {}),
    )


def test_get_build_snaps(part_info):
    properties = KernelPlugin.properties_class.unmarshal({"source": "."})
    plugin = KernelPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_snaps() == set()


def test_get_build_packages(part_info):
    properties = KernelPlugin.properties_class.unmarshal({"source": "."})
    plugin = KernelPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_packages() == {
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


def test_get_build_packages_zfs(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {"source": ".", "kernel-enable-zfs-support": "true"}
    )
    plugin = KernelPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_packages() == {
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
        "autoconf",
        "automake",
        "libblkid-dev",
        "libtool",
        "python3",
    }


def test_get_build_environment(part_info):
    properties = KernelPlugin.properties_class.unmarshal({"source": "."})
    plugin = KernelPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_environment() == {
        "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
        "ARCH": "x86",
        "KERNEL_IMAGE": "bzImage",
        "KERNEL_TARGET": "modules",
    }


def test_get_build_commands(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {
            "source": ".",
            "kernel-kdefconfig": [
                "snappy_defconfig",
                "foo_config",
            ],
            "kernel-kconfigs": [
                "CONFIG_FOO=y",
                "CONFIG_BAR=m",
            ],
            "kernel-enable-zfs-support": "true",
            "kernel-enable-perf": "true",
        }
    )
    plugin = KernelPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/kernel_build.sh "
        "kernel-kconfigflavour= "
        "kernel-kdefconfig=snappy_defconfig,foo_config "
        "kernel-kconfigs=CONFIG_FOO=y,CONFIG_BAR=m "
        "kernel-enable-zfs=True "
        "kernel-enable-perf=True"
    ]
