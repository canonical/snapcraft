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

import inspect
import logging
import platform
import sys
import tempfile
import textwrap
from dataclasses import dataclass
from unittest import mock

import pytest
from testtools import TestCase

from snapcraft_legacy.plugins.v2._kernel_build import check_new_config
from snapcraft_legacy.plugins.v2.kernel import KernelPlugin


class Kernelv2PluginProperties:
    kernel_kdefconfig: [str] = ["defconfig"]
    kernel_kconfigflavour: str = None
    kernel_kconfigs: [str] = None
    kernel_image_target = "Image.gz"
    kernel_enable_zfs_support: bool = False
    kernel_enable_perf: bool = False
    kernel_add_ppa: bool = False


class TestPluginKernel(TestCase):
    """
    Legacy kernel plugin tests.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _setup_test(
        self,
        kernelkdefconfig=["defconfig"],
        kernelkconfigflavour=None,
        kernelkconfigs=None,
        kernelimagetarget="Image.gz",
        kernelenablezfssupport=False,
        kernelenableperf=False,
        arch=platform.machine(),
    ):
        @dataclass
        class Options(Kernelv2PluginProperties):
            kernel_kdefconfig = kernelkdefconfig
            kernel_kconfigflavour = kernelkconfigflavour
            kernel_kconfigs = kernelkconfigs
            kernel_image_target = kernelimagetarget
            kernel_enable_zfs_support = kernelenablezfssupport
            kernel_enable_perf = kernelenableperf
            # Ensure that the PPA is not added so we don't cause side-effects
            kernel_add_ppa = False

        target_arch = _DEB_ARCH_TRANSLATIONS[arch]

        with mock.patch(
            "snapcraft_legacy.plugins.v2.kernel._get_target_architecture",
            return_value=target_arch,
        ):
            plugin = KernelPlugin(part_name="kernel", options=Options())

        # Snapcraft is broken to report correctly target arch. Make sure the plugin is set correctly.
        # In normal execution the plugin detects cross build based on the snapcraft command line.
        if arch != platform.machine():
            plugin._cross_building = True
            plugin._target_arch = target_arch
            plugin._kernel_arch = _KERNEL_ARCH_TRANSLATIONS[arch]
            plugin._deb_arch = _DEB_ARCH_TRANSLATIONS[arch]
        else:
            plugin._cross_building = False

        return plugin

    def test_schema(self):
        self.assertEqual(
            KernelPlugin.get_schema(),
            {
                "$schema": "http://json-schema.org/draft-04/schema#",
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "kernel-kdefconfig": {
                        "type": "array",
                        "default": [],
                    },
                    "kernel-kconfigflavour": {"type": "string", "default": "defconfig"},
                    "kernel-kconfigs": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-image-target": {
                        "oneOf": [{"type": "string"}, {"type": "object"}],
                        "default": "",
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
            },
        )

    def test_get_base_build_packages(self):
        plugin = self._setup_test()
        # default initrd compression is zstd
        self.assertEqual(
            plugin.get_build_packages(),
            {
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
            },
        )

    def test_get_base_build_packages_zfs(self):
        plugin = self._setup_test(kernelenablezfssupport=True)
        self.assertEqual(
            plugin.get_build_packages(),
            {
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
                "autoconf",
                "automake",
                "libblkid-dev",
                "libtool",
                "python3",
            },
        )

    def test_get_base_build_packages_zfs_cross(self):
        plugin = self._setup_test(kernelenablezfssupport=True, arch="armv7l")
        self.assertEqual(
            plugin.get_build_packages(),
            {
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
                "autoconf",
                "automake",
                "libblkid-dev",
                "libtool",
                "python3",
                "libc6-dev:armhf",
            },
        )

    def test_get_build_snaps(self):
        plugin = self._setup_test()
        self.assertEqual(plugin.get_build_snaps(), set())

    def test_check_configuration_simple(self):
        plugin = self._setup_test()
        opt = plugin.options

        self.assertEqual(opt.kernel_kdefconfig, ["defconfig"])
        self.assertEqual(opt.kernel_kconfigflavour, None)
        self.assertEqual(opt.kernel_kconfigs, None)
        self.assertEqual(opt.kernel_image_target, "Image.gz")
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)

    def test_check_configuration_kde_config(self):
        plugin = self._setup_test(
            kernelkdefconfig=["snappy_defconfig"],
            kernelimagetarget="Image",
        )
        opt = plugin.options

        self.assertEqual(opt.kernel_kdefconfig, ["snappy_defconfig"])
        self.assertIs(opt.kernel_kconfigflavour, None)
        self.assertIs(opt.kernel_kconfigs, None)
        self.assertEqual(opt.kernel_image_target, "Image")
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)

    def test_check_configuration_image_target(self):
        plugin = self._setup_test(
            kernelkdefconfig=["snappy_defconfig"],
            kernelimagetarget={"arm64": "Image", "armhf": "Image.gz"},
        )
        opt = plugin.options

        self.assertEqual(opt.kernel_kdefconfig, ["snappy_defconfig"])
        self.assertIs(opt.kernel_kconfigflavour, None)
        self.assertIs(opt.kernel_kconfigs, None)
        self.assertEqual(
            opt.kernel_image_target, {"arm64": "Image", "armhf": "Image.gz"}
        )
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)

    def test_check_get_build_environment(self):
        plugin = self._setup_test()
        plugin._kernel_arch = "amd64"

        self.assertEqual(
            plugin.get_build_environment(),
            {
                "CROSS_COMPILE": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
                "ARCH": plugin._kernel_arch,
                "DEB_ARCH": "${SNAPCRAFT_TARGET_ARCH}",
                "KERNEL_BUILD_ARCH_DIR": f"${{SNAPCRAFT_PART_BUILD}}/arch/{plugin._kernel_arch}/boot",
                "KERNEL_IMAGE_TARGET": plugin.kernel_image_target,
            },
        )

    def test_check_get_build_environment_cross(self):
        plugin = self._setup_test(
            kernelimagetarget={"arm64": "Image", "armhf": "Image.gz"},
            arch="armv7l",
        )

        plugin._set_kernel_targets()
        self.assertEqual(
            plugin.get_build_environment(),
            {
                "CROSS_COMPILE": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
                "ARCH": "arm",
                "DEB_ARCH": "${SNAPCRAFT_TARGET_ARCH}",
                "KERNEL_BUILD_ARCH_DIR": "${SNAPCRAFT_PART_BUILD}/arch/arm/boot",
                "KERNEL_IMAGE_TARGET": "Image.gz",
            },
        )

    def test_check_get_build_command(self):
        plugin = self._setup_test()

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert not _is_sub_array(build_commands, _clone_zfs_cmd)
        assert _is_sub_array(build_commands, _clean_old_build_cmd)
        assert _is_sub_array(build_commands, _prepare_config_cmd)
        assert _is_sub_array(build_commands, _remake_old_config_cmd)
        assert _check_config in build_commands
        if platform.machine() == "x86_64":
            assert _is_sub_array(build_commands, _build_kernel_x86_cmd)
            assert _is_sub_array(build_commands, _install_kernel_x86_cmd)
        else:
            assert _is_sub_array(build_commands, _build_kernel_cmd)
            assert _is_sub_array(build_commands, _install_kernel_cmd)

        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert not _is_sub_array(build_commands, _build_zfs_cmd)
        assert not _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_config_flavour_configs(self):
        plugin = self._setup_test(
            kernelkconfigflavour="raspi",
            kernelkconfigs=["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
            kernelenablezfssupport=True,
            kernelenableperf=True,
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert _is_sub_array(build_commands, _clone_zfs_cmd)
        assert _is_sub_array(build_commands, _clean_old_build_cmd)
        assert _is_sub_array(build_commands, _prepare_config_flavour_cmd)
        assert _is_sub_array(build_commands, _prepare_config_extra_config_cmd)
        assert _is_sub_array(build_commands, _remake_old_config_cmd)
        assert _check_config in build_commands
        if platform.machine() == "x86_64":
            assert _is_sub_array(build_commands, _build_kernel_x86_cmd)
            assert _is_sub_array(build_commands, _install_kernel_no_dtbs_cmd)
        else:
            assert _is_sub_array(build_commands, _build_kernel_cmd)
            assert _is_sub_array(build_commands, _install_kernel_dtbs_cmd)

        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert _is_sub_array(build_commands, _build_zfs_cmd)
        assert _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_cross(self):
        plugin = self._setup_test(
            kernelkconfigflavour="raspi",
            kernelkconfigs=["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
            kernelimagetarget={"arm64": "Image", "armhf": "Image.gz"},
            kernelenablezfssupport=True,
            kernelenableperf=True,
            arch="armv7l",
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert _is_sub_array(build_commands, _clone_zfs_cmd)
        assert _is_sub_array(build_commands, _clean_old_build_cmd)
        assert _is_sub_array(build_commands, _prepare_config_flavour_cmd)
        assert _is_sub_array(build_commands, _prepare_config_extra_config_cmd)
        assert _is_sub_array(build_commands, _remake_old_config_armhf_cmd)
        assert _check_config in build_commands
        assert _is_sub_array(build_commands, _build_kernel_armhf_cmd)
        assert _is_sub_array(build_commands, _install_kernel_armhf_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert _is_sub_array(build_commands, _build_zfs_cmd)
        assert _is_sub_array(build_commands, _build_perf_armhf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_arch_aarch64(self):
        arch = "aarch64"
        cross_building = False
        if platform.machine() != arch:
            cross_building = True
        plugin = self._setup_test(arch=arch)

        assert plugin._target_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin._deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_armhf(self):
        arch = "armv7l"
        cross_building = False
        if platform.machine() != arch:
            cross_building = True
        plugin = self._setup_test(arch=arch)

        assert plugin._target_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin._deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_riscv64(self):
        arch = "riscv64"
        cross_building = False
        if platform.machine() != arch:
            cross_building = True
        plugin = self._setup_test(arch=arch)

        assert plugin._target_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin._deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_x86_64(self):
        arch = "x86_64"
        cross_building = False
        if platform.machine() != arch:
            cross_building = True
        plugin = self._setup_test(arch=arch)

        assert plugin._target_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin._deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_i686(self):
        # we do not support i686 so use this arch to test unknnown arch by plugin
        arch = "i686"
        with pytest.raises(ValueError) as error:
            self._setup_test(arch=arch)

        assert str(error.value) == "unknown deb architecture"

    def test_check_new_config_good(self):
        # create test config
        with tempfile.NamedTemporaryFile(mode="w+", encoding="utf-8") as config_file:
            config_file.write(
                "".join(
                    [
                        x + "\n"
                        for x in _BUUILT_IN_BASE
                        + _SECCOMP_BUILD_IN
                        + _SQUASHFS_BUUILT_IN
                        + _SQUASHFS_LZO_BUILT_IN
                    ]
                )
            )
            config_file.flush()
            with self.assertLogs(level=logging.WARNING) as cm:
                # assert log does not support no log case, we log and check that
                # if the only log
                logging.getLogger("kernel.py").warning("ONLY WARNING")
                check_new_config(config_path=config_file.name)
            self.assertEqual(cm.output, ["WARNING:kernel.py:ONLY WARNING"])

    def test_check_new_config_missing(self):
        # create test config
        with tempfile.NamedTemporaryFile(mode="w+", encoding="utf-8") as config_file:
            config_file.write(
                "".join(
                    [
                        x + "\n"
                        for x in _BUUILT_IN_BASE
                        + _SECCOMP_NOT_SET
                        + _SQUASHFS_NOT_SET
                        + _SQUASHFS_LZO_NOT_SET
                    ]
                )
            )
            config_file.flush()
            with self.assertLogs(level=logging.WARNING) as cm:
                check_new_config(config_path=config_file.name)
            # there should be 2 warning logs, one for missing configs, one for kernel module
            assert len(cm.output) == 2
            assert (
                "**** WARNING **** WARNING **** WARNING **** WARNING ****"
                in cm.output[0]
            )
            assert "CONFIG_SECCOMP" in cm.output[0]
            assert "CONFIG_SQUASHFS" in cm.output[0]
            assert (
                "CONFIG_SQUASHFS_LZO (used by desktop snaps for accelerated loading)"
                in cm.output[0]
            )
            assert (
                "**** WARNING **** WARNING **** WARNING **** WARNING ****"
                in cm.output[1]
            )
            assert "missing options:\nCONFIG_SQUASHFS\n" in cm.output[1]

    def test_check_new_config_squash_module_missing(self):
        # create test config
        with tempfile.NamedTemporaryFile(mode="w+", encoding="utf-8") as config_file:
            config_file.write(
                "".join(
                    [
                        x + "\n"
                        for x in _BUUILT_IN_BASE
                        + _SECCOMP_BUILD_IN
                        + _SQUASHFS_AS_MODULE
                        + _SQUASHFS_LZO_BUILT_IN
                    ]
                )
            )
            config_file.flush()
            with self.assertLogs(level=logging.WARNING) as cm:
                check_new_config(config_path=config_file.name)
            # there should be 1 warning log for missing module in initrd
            assert len(cm.output) == 1
            assert (
                "**** WARNING **** WARNING **** WARNING **** WARNING ****"
                in cm.output[0]
            )
            assert "adding\nthe corresponding module to initrd:\n" in cm.output[0]
            assert (
                "modules to be included in the initrd:\nCONFIG_SQUASHFS" in cm.output[0]
            )

    def test_check_new_config_squash_missing_file(self):
        # run with invalid file
        e = ""
        try:
            check_new_config(config_path="wrong/file")
        except FileNotFoundError as err:
            e = err.strerror
        assert e == "No such file or directory"


def _is_sub_array(array, sub_array):
    a = len(array)
    s = len(sub_array)
    # Two pointers to traverse the arrays
    i = 0
    j = 0

    # Traverse both arrays simultaneously
    while i < a and j < s:
        # If element matches
        # increment both pointers
        if array[i] == sub_array[j]:
            i += 1
            j += 1

            # If sub_array is completely traversed
            if j == s:
                return True

        # If not, increment i and reset j
        else:
            i = i - j + 1
            j = 0

    return False


_DEB_ARCH_TRANSLATIONS = {
    "aarch64": "arm64",
    "armv7l": "armhf",
    "i686": "i386",
    "riscv64": "riscv64",
    "x86_64": "amd64",
}

_KERNEL_ARCH_TRANSLATIONS = {
    "aarch64": "arm64",
    "armv7l": "arm",
    "i686": "x86",
    "riscv64": "riscv",
    "x86_64": "x86",
}

_determine_kernel_src = [
    " ".join(
        [
            "[ -d ${SNAPCRAFT_PART_SRC}/kernel ]",
            "&&",
            "KERNEL_SRC=${SNAPCRAFT_PART_SRC}",
            "||",
            "KERNEL_SRC=${SNAPCRAFT_PROJECT_DIR}",
        ],
    ),
    'echo "PATH=$PATH"',
    'echo "KERNEL_SRC=${KERNEL_SRC}"',
]

_clean_old_build_cmd = [
    textwrap.dedent(
        """
        echo "Cleaning previous build first..."
        [ -e ${SNAPCRAFT_PART_INSTALL}/modules ] && rm -rf ${SNAPCRAFT_PART_INSTALL}/modules
        [ -L ${SNAPCRAFT_PART_INSTALL}/lib/modules ] && rm -rf ${SNAPCRAFT_PART_INSTALL}/lib/modules
        """
    )
]


_prepare_config_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${SNAPCRAFT_PART_BUILD}/.config ]; then",
    "\tmake -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} defconfig",
    "fi",
]

_prepare_config_custom_cc_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${SNAPCRAFT_PART_BUILD}/.config ]; then",
    '\tmake -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} CC="my-gcc" defconfig',
    "fi",
]

_prepare_config_flavour_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${SNAPCRAFT_PART_BUILD}/.config ]; then",
    textwrap.dedent(
        """	echo "Assembling Ubuntu config..."
	if [ -f ${KERNEL_SRC}/debian/rules ] && [ -x ${KERNEL_SRC}/debian/rules ]; then
		# Generate Ubuntu kernel configs
		pushd ${KERNEL_SRC}
		fakeroot debian/rules clean genconfigs || true
		popd

		# Pick the right kernel .config for the target arch and flavour
		ubuntuconfig=${KERNEL_SRC}/CONFIGS/${DEB_ARCH}-config.flavour.raspi
		cat ${ubuntuconfig} > ${SNAPCRAFT_PART_BUILD}/.config

		# Clean up kernel source directory
		pushd ${KERNEL_SRC}
		fakeroot debian/rules clean
		rm -rf CONFIGS/
		popd
	fi"""
    ),
    "fi",
]

_prepare_config_extra_config_cmd = [
    'echo "Applying extra config...."',
    "echo 'CONFIG_DEBUG_INFO=n\nCONFIG_DM_CRYPT=y' > ${SNAPCRAFT_PART_BUILD}/.config_snap",
    "cat ${SNAPCRAFT_PART_BUILD}/.config >> ${SNAPCRAFT_PART_BUILD}/.config_snap",
    "echo 'CONFIG_DEBUG_INFO=n\nCONFIG_DM_CRYPT=y' >> ${SNAPCRAFT_PART_BUILD}/.config_snap",
    "mv ${SNAPCRAFT_PART_BUILD}/.config_snap ${SNAPCRAFT_PART_BUILD}/.config",
]
_remake_old_config_cmd = [
    'echo "Remaking oldconfig...."',
    "bash -c 'yes \"\" || true' | make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} oldconfig",
]

_remake_old_config_armhf_cmd = [
    'echo "Remaking oldconfig...."',
    " ".join(
        [
            "bash -c 'yes \"\" || true' |",
            "make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD}",
            "ARCH=arm CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "oldconfig",
        ],
    ),
]

_check_config = " ".join(
    [
        sys.executable,
        "-I",
        inspect.getfile(check_new_config),
        "check_new_config",
        "${SNAPCRAFT_PART_BUILD}/.config",
    ],
)

_build_kernel_cmd = [
    "make -j$(nproc) -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} Image.gz modules dtbs",
]

_build_kernel_x86_cmd = [
    "make -j$(nproc) -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} Image.gz modules",
]

_build_kernel_armhf_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "ARCH=arm",
            "CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "Image.gz modules dtbs",
        ],
    ),
]

_install_kernel_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${SNAPCRAFT_PART_INSTALL}/dtbs",
        ],
    ),
]

_install_kernel_x86_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
        ],
    ),
]

_install_kernel_no_dtbs_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
        ],
    ),
]

_install_kernel_dtbs_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${SNAPCRAFT_PART_INSTALL}/dtbs",
        ],
    ),
]

_install_kernel_armhf_cmd = [
    " ".join(
        [
            "make",
            "-j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "ARCH=arm",
            "CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${SNAPCRAFT_PART_INSTALL}/dtbs",
        ],
    ),
]

_parse_kernel_release_cmd = [
    "KERNEL_RELEASE=$(cat ${SNAPCRAFT_PART_BUILD}/include/config/kernel.release)",
]

_install_config_cmd = [
    'echo "Installing kernel config..."',
    "ln -f ${SNAPCRAFT_PART_BUILD}/.config ${SNAPCRAFT_PART_INSTALL}/config-${KERNEL_RELEASE}",
]

_finalize_install_cmd = [
    textwrap.dedent(
        """

        echo "Finalizing install directory..."
        # upstream kernel installs under $INSTALL_MOD_PATH/lib/modules/
        # but snapd expects modules/ and firmware/
        mv ${SNAPCRAFT_PART_INSTALL}/lib/modules ${SNAPCRAFT_PART_INSTALL}/
        # remove symlinks modules/*/build and modules/*/source
        rm -rf ${SNAPCRAFT_PART_INSTALL}/modules/*/build ${SNAPCRAFT_PART_INSTALL}/modules/*/source
        # if there is firmware dir, move it to snap root
        # this could have been from stage packages or from kernel build
        [ -d ${SNAPCRAFT_PART_INSTALL}/lib/firmware ] && mv ${SNAPCRAFT_PART_INSTALL}/lib/firmware ${SNAPCRAFT_PART_INSTALL}
        # create symlinks for modules and firmware for convenience
        ln -sf ../modules ${SNAPCRAFT_PART_INSTALL}/lib/modules
        ln -sf ../firmware ${SNAPCRAFT_PART_INSTALL}/lib/firmware
        """
    )
]

_clone_zfs_cmd = [
    textwrap.dedent(
        """
        if [ ! -d ${SNAPCRAFT_PART_BUILD}/zfs ]; then
        	echo "cloning zfs..."
        	git clone --depth=1 https://github.com/openzfs/zfs ${SNAPCRAFT_PART_BUILD}/zfs -b master
        fi
        """
    )
]

_build_zfs_cmd = [
    textwrap.dedent(
        """
        echo "Building zfs modules..."
        cd ${SNAPCRAFT_PART_BUILD}/zfs
        ./autogen.sh
        ./configure --with-linux=${KERNEL_SRC} --with-linux-obj=${SNAPCRAFT_PART_BUILD} \
--with-config=kernel --host=${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}
        make -j$(nproc)
        make install DESTDIR=${SNAPCRAFT_PART_INSTALL}/zfs
        release_version="$(ls ${SNAPCRAFT_PART_INSTALL}/modules)"
        mv ${SNAPCRAFT_PART_INSTALL}/zfs/lib/modules/${release_version}/extra \
${SNAPCRAFT_PART_INSTALL}/modules/${release_version}
        rm -rf ${SNAPCRAFT_PART_INSTALL}/zfs
        echo "Rebuilding module dependencies"
        depmod -b ${SNAPCRAFT_PART_INSTALL} ${release_version}
        """
    )
]

_build_perf_cmd = [
    'echo "Building perf binary..."',
    'mkdir -p "${SNAPCRAFT_PART_BUILD}/tools/perf"',
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            '-C "${KERNEL_SRC}/tools/perf"',
            'O="${SNAPCRAFT_PART_BUILD}/tools/perf"',
        ],
    ),
    'install -Dm0755 "${SNAPCRAFT_PART_BUILD}/tools/perf/perf" "${SNAPCRAFT_PART_INSTALL}/bin/perf"',
]

_build_perf_armhf_cmd = [
    'echo "Building perf binary..."',
    'mkdir -p "${SNAPCRAFT_PART_BUILD}/tools/perf"',
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "ARCH=arm",
            "CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            '-C "${KERNEL_SRC}/tools/perf"',
            'O="${SNAPCRAFT_PART_BUILD}/tools/perf"',
        ],
    ),
    'install -Dm0755 "${SNAPCRAFT_PART_BUILD}/tools/perf/perf" "${SNAPCRAFT_PART_INSTALL}/bin/perf"',
]

_BUUILT_IN_BASE = [
    "CONFIG_DEVTMPFS=y",
    "CONFIG_DEVTMPFS_MOUNT=y",
    "CONFIG_TMPFS_POSIX_ACL=y",
    "CONFIG_IPV6=y",
    "CONFIG_SYSVIPC=y",
    "CONFIG_SYSVIPC_SYSCTL=y",
    "CONFIG_VFAT_FS=y",
    "CONFIG_NLS_CODEPAGE_437=y",
    "CONFIG_NLS_ISO8859_1=y",
    "CONFIG_SECURITY=y",
    "CONFIG_SECURITY_APPARMOR=y",
    "CONFIG_SYN_COOKIES=y",
    "CONFIG_STRICT_DEVMEM=y",
    "CONFIG_DEFAULT_SECURITY_APPARMOR=y",
    "CONFIG_SECCOMP_FILTER=y",
    "CONFIG_RD_LZMA=y",
    "CONFIG_KEYS=y",
    "CONFIG_ENCRYPTED_KEYS=y",
    "CONFIG_DEVTMPFS=y",
    "CONFIG_CGROUPS=y",
    "CONFIG_INOTIFY_USER=y",
    "CONFIG_SIGNALFD=y",
    "CONFIG_TIMERFD=y",
    "CONFIG_EPOLL=y",
    "CONFIG_NET=y",
    "CONFIG_SYSFS=y",
    "CONFIG_PROC_FS=y",
    "CONFIG_FHANDLE=y",
    "CONFIG_BLK_DEV_BSG=y",
    "CONFIG_NET_NS=y",
    "CONFIG_IPV6=y",
    "CONFIG_AUTOFS4_FS=y",
    "CONFIG_TMPFS_POSIX_ACL=y",
    "CONFIG_TMPFS_XATTR=y",
    "CONFIG_SQUASHFS_XATTR=y",
    "CONFIG_SQUASHFS_XZ=y",
]

_SQUASHFS_LZO_NOT_SET = ["# CONFIG_SQUASHFS_LZO is not set"]

_SQUASHFS_LZO_BUILT_IN = ["CONFIG_SQUASHFS_LZO=y"]

_SECCOMP_NOT_SET = ["# CONFIG_SECCOMP is not set"]

_SECCOMP_BUILD_IN = ["CONFIG_SECCOMP=y"]

_SQUASHFS_NOT_SET = ["# CONFIG_SQUASHFS is not set"]

_SQUASHFS_BUUILT_IN = ["CONFIG_SQUASHFS=y"]

_SQUASHFS_AS_MODULE = ["CONFIG_SQUASHFS=m"]
