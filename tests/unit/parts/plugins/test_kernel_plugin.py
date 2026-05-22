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
from craft_parts import Part, PartInfo, ProjectInfo, errors

from snapcraft.parts.plugins import KernelPlugin


@pytest.fixture(autouse=True)
def part_info(new_dir):
    yield PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="core22",
            cache_dir=new_dir,
        ),
        part=Part("my-part", {}),
    )


def test_validate_release_name_and_source_exclusive():
    with pytest.raises(errors.PartsError):
        KernelPlugin.properties_class.unmarshal(
            {
                "kernel-ubuntu-release-name": "noble",
                "source": "https://github.com/example/kernel",
            }
        )


def test_validate_debian_and_binary_exclusive():
    with pytest.raises(errors.PartsError):
        KernelPlugin.properties_class.unmarshal(
            {
                "kernel-ubuntu-binary-package": True,
                "kernel-ubuntu-debian-package": True,
            }
        )


def test_validate_binary_and_tools_exclusive():
    with pytest.raises(errors.PartsError):
        KernelPlugin.properties_class.unmarshal(
            {
                "kernel-ubuntu-binary-package": True,
                "kernel-tools": ["bpftool"],
            }
        )


def test_validate_debian_dkms_requires_debian_package():
    with pytest.raises(errors.PartsError):
        KernelPlugin.properties_class.unmarshal(
            {
                "kernel-ubuntu-debian-dkms": ["nvidia-dkms-535"],
            }
        )


def test_validate_debian_package_and_kconfigs_options_exclusive(emitter):
    KernelPlugin.properties_class.unmarshal(
        {
            "kernel-ubuntu-debian-package": True,
            "kernel-kconfigs": ["CONFIG_FOO=y"],
        }
    )
    emitter.assert_progress(
        "'kernel-kconfigs' will be ignored when 'kernel-ubuntu-debian-package' is set",
        permanent=True,
    )


def test_get_pull_commands_release(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {
            "kernel-ubuntu-release-name": "jammy",
            "kernel-ubuntu-abinumber": "Ubuntu-5.15.0-176.186",
        }
    )
    plugin = KernelPlugin(properties=properties, part_info=part_info)

    expected_commands = [
        "for name in canonical-kernel ubuntu-kernel; do",
        'team_url="https://git.launchpad.net/~$name/ubuntu/+source/linux/+git/jammy"',
        'test_url="$(curl -fsSL "$team_url")"',
        'if [ -n "$test_url" ] && [ "$test_url" != "Invalid OpenID transaction" ]; then',
        'actual_url="$team_url"',
        "fi",
        "done",
        "git init",
        'git remote add origin "$actual_url"',
        "git fetch --depth 1 origin Ubuntu-5.15.0-176.186",
        "git checkout FETCH_HEAD",
    ]

    assert plugin.get_pull_commands() == expected_commands


def test_get_build_snaps(part_info):
    properties = KernelPlugin.properties_class.unmarshal({})
    plugin = KernelPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_snaps() == set()


def test_get_build_packages_core22(part_info, new_dir):
    part_info = PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="core22",
            cache_dir=new_dir,
        ),
        part=Part("my-part", {}),
    )

    properties = KernelPlugin.properties_class.unmarshal({})
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
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libelf-dev",
        "libssl-dev",
        "lz4",
        "systemd",
        "xz-utils",
        "zstd",
    }


def test_get_build_packages_core24(part_info, new_dir):
    part_info = PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="core24",
            cache_dir=new_dir,
        ),
        part=Part("my-part", {}),
    )

    properties = KernelPlugin.properties_class.unmarshal({})
    plugin = KernelPlugin(properties=properties, part_info=part_info)
    assert plugin.get_build_packages() == {
        "bc",
        "binutils",
        "bison",
        "clang",
        "cmake",
        "cpio",
        "cryptsetup",
        "debhelper",
        "fakeroot",
        "flex",
        "gawk",
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libdw-dev",
        "libelf-dev",
        "libssl-dev",
        "llvm",
        "lz4",
        "rustc",
        "systemd",
        "xz-utils",
        "zstd",
    }


def test_get_build_packages_bpftool(part_info):
    properties = KernelPlugin.properties_class.unmarshal({"kernel-tools": ["bpftool"]})
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
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libelf-dev",
        "libssl-dev",
        "lz4",
        "systemd",
        "xz-utils",
        "zstd",
        "libelf-dev:amd64",
        "zlib1g-dev:amd64",
        "libcap-dev",
    }


def test_get_build_packages_cpupower(part_info):
    properties = KernelPlugin.properties_class.unmarshal({"kernel-tools": ["cpupower"]})
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
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libelf-dev",
        "libssl-dev",
        "lz4",
        "systemd",
        "xz-utils",
        "zstd",
        "libpci-dev:amd64",
    }


def test_get_build_packages_perf(part_info):
    properties = KernelPlugin.properties_class.unmarshal({"kernel-tools": ["perf"]})
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
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libcap-dev",
        "libelf-dev",
        "libiberty-dev:amd64",
        "libssl-dev",
        "libtraceevent-dev:amd64",
        "lz4",
        "systemd",
        "xz-utils",
        "zstd",
    }


def test_get_build_packages_bpftool_perf_libcap(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {"kernel-tools": ["bpftool", "perf"]}
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
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libcap-dev",
        "libelf-dev",
        "libelf-dev:amd64",
        "libiberty-dev:amd64",
        "libssl-dev",
        "libtraceevent-dev:amd64",
        "lz4",
        "systemd",
        "xz-utils",
        "zlib1g-dev:amd64",
        "zstd",
    }


def test_get_build_packages_debian_package_tools_rsync(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {
            "kernel-ubuntu-debian-package": True,
            "kernel-tools": ["bpftool"],
        }
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
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libcap-dev",
        "libelf-dev",
        "libelf-dev:amd64",
        "libssl-dev",
        "lz4",
        "rsync",
        "systemd",
        "xz-utils",
        "zlib1g-dev:amd64",
        "zstd",
    }


def test_get_build_packages_debian_dkms_adds_dkms(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {
            "kernel-ubuntu-debian-package": True,
            "kernel-ubuntu-debian-dkms": ["nvidia-dkms-535"],
        }
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
        "gcc-x86-64-linux-gnu",
        "kmod",
        "kpartx",
        "libelf-dev",
        "libssl-dev",
        "lz4",
        "systemd",
        "xz-utils",
        "zstd",
        "dkms",
    }


def test_get_build_environment(part_info):
    properties = KernelPlugin.properties_class.unmarshal({})
    plugin = KernelPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_environment() == {
        "CROSS": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
        "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
        "ARCH": "x86",
        "KERNEL_IMAGE": "bzImage",
        "KERNEL_TARGET": "modules",
    }


def test_get_build_commands_binary_package(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {
            "kernel-ubuntu-binary-package": True,
        }
    )
    plugin = KernelPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/kernel_build.sh "
        "kernel-kdefconfig='defconfig' "
        "kernel-kconfigs= "
        "kernel-tools='' "
        "kernel-ubuntu-kconfigflavour=generic "
        "kernel-ubuntu-release-name=None "
        "kernel-ubuntu-abinumber= "
        "kernel-ubuntu-binary-package=True "
        "kernel-ubuntu-debian-package=False "
        "kernel-ubuntu-debian-dkms=''"
    ]


def test_get_build_commands_debian_package(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {
            "kernel-ubuntu-debian-package": True,
            "kernel-ubuntu-debian-dkms": [
                "nvidia-dkms-535",
                "nvidia-kernel-source-535",
            ],
        }
    )
    plugin = KernelPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/kernel_build.sh "
        "kernel-kdefconfig='defconfig' "
        "kernel-kconfigs= "
        "kernel-tools='' "
        "kernel-ubuntu-kconfigflavour=generic "
        "kernel-ubuntu-release-name=None "
        "kernel-ubuntu-abinumber= "
        "kernel-ubuntu-binary-package=False "
        "kernel-ubuntu-debian-package=True "
        "kernel-ubuntu-debian-dkms='nvidia-dkms-535 nvidia-kernel-source-535'"
    ]


def test_get_build_commands(part_info):
    properties = KernelPlugin.properties_class.unmarshal(
        {
            "kernel-kdefconfig": [
                "snappy_defconfig",
                "foo_config",
            ],
            "kernel-kconfigs": [
                "CONFIG_FOO=y",
                "CONFIG_BAR=m",
            ],
            "kernel-tools": [
                "bpftool",
                "cpupower",
                "perf",
            ],
            "kernel-ubuntu-release-name": "noble",
            "kernel-ubuntu-debian-package": "false",
        }
    )
    plugin = KernelPlugin(properties=properties, part_info=part_info)

    assert plugin.get_build_commands() == [
        "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/kernel_build.sh "
        "kernel-kdefconfig='snappy_defconfig foo_config' "
        "kernel-kconfigs=CONFIG_FOO=y,CONFIG_BAR=m "
        "kernel-tools='bpftool cpupower perf' "
        "kernel-ubuntu-kconfigflavour= "
        "kernel-ubuntu-release-name=noble "
        "kernel-ubuntu-abinumber= "
        "kernel-ubuntu-binary-package=False "
        "kernel-ubuntu-debian-package=False "
        "kernel-ubuntu-debian-dkms=''"
    ]
