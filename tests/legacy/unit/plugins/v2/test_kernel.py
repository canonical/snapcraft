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
from dataclasses import dataclass
from typing import Union

from testtools import TestCase

from snapcraft_legacy.plugins.v2.kernel import KernelPlugin, check_new_config


class Kernelv2PluginProperties:
    kernel_kdefconfig: [str] = ["defconfig"]
    kernel_kconfigfile: str = None
    kernel_kconfigflavour: str = None
    kernel_kconfigs: [str] = None
    kernel_image_target = "Image.gz"
    kernel_with_firmware: bool = True
    kernel_device_trees: [str] = None
    kernel_compiler: str = None
    kernel_compiler_paths: [str] = None
    kernel_compiler_parameters: [str] = None
    kernel_initrd_modules: [str] = None
    kernel_initrd_configured_modules: [str] = None
    kernel_initrd_firmware: [str] = None
    kernel_initrd_compression: [str] = None
    kernel_initrd_compression_options: [str] = None
    kernel_initrd_overlay: str = None
    kernel_initrd_addons: [str] = None
    kernel_enable_zfs_support: bool = False
    kernel_enable_perf: bool = False
    kernel_add_ppa: bool = False
    kernel_use_llvm: bool = False


class TestPluginKernel(TestCase):
    """
    Legacy kernel plugin tests.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _setup_test(
        self,
        kernelkdefconfig=["defconfig"],
        kernelkconfigfile=None,
        kernelkconfigflavour=None,
        kernelkconfigs=None,
        kernelimagetarget="Image.gz",
        kernelwithfirmware=True,
        kerneldevicetrees=None,
        kernelinitrdmodules=None,
        kernelinitrdconfiguredmodules=None,
        kernelinitrdstagefirmware=False,
        kernelinitrdfirmware=None,
        kernelinitrdcompression=None,
        kernelinitrdcompressionoptions=None,
        kernelinitrdoverlay=None,
        kernelinitrdaddons=None,
        kernelcompiler=None,
        kernelcompilerpaths=None,
        kernelcompilerparameters=None,
        kernelenablezfssupport=False,
        kernelenableperf=False,
        kernelusellvm: Union[bool, str] = False,
        arch=platform.machine(),
    ):
        @dataclass
        class Options(Kernelv2PluginProperties):
            kernel_kdefconfig = kernelkdefconfig
            kernel_kconfigfile = kernelkconfigfile
            kernel_kconfigflavour = kernelkconfigflavour
            kernel_kconfigs = kernelkconfigs
            kernel_image_target = kernelimagetarget
            kernel_with_firmware = kernelwithfirmware
            kernel_device_trees = kerneldevicetrees
            kernel_initrd_modules = kernelinitrdmodules
            kernel_initrd_configured_modules = kernelinitrdconfiguredmodules
            kernel_initrd_stage_firmware = kernelinitrdstagefirmware
            kernel_initrd_firmware = kernelinitrdfirmware
            kernel_initrd_compression = kernelinitrdcompression
            kernel_initrd_compression_options = kernelinitrdcompressionoptions
            kernel_initrd_overlay = kernelinitrdoverlay
            kernel_initrd_addons = kernelinitrdaddons
            kernel_compiler = kernelcompiler
            kernel_compiler_paths = kernelcompilerpaths
            kernel_compiler_parameters = kernelcompilerparameters
            kernel_enable_zfs_support = kernelenablezfssupport
            kernel_enable_perf = kernelenableperf
            kernel_use_llvm = kernelusellvm
            # Ensure that the PPA is not added so we don't cause side-effects
            kernel_add_ppa = False

        plugin = KernelPlugin(part_name="kernel", options=Options())

        # snapcraft if broken to report correctly target arch, make sure plugin is set correctly
        # in normal exuction plugin detects cross build based on the command invoking snapcraft
        if arch != platform.machine():
            plugin._cross_building = True
            plugin.target_arch = _DEB_ARCH_TRANSLATIONS[arch]
            plugin.kernel_arch = _KERNEL_ARCH_TRANSLATIONS[arch]
            plugin.deb_arch = _DEB_ARCH_TRANSLATIONS[arch]
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
                        "default": ["defconfig"],
                    },
                    "kernel-kconfigfile": {
                        "type": "string",
                        "default": None,
                    },
                    "kernel-kconfigflavour": {"type": "string", "default": ""},
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
                    "kernel-with-firmware": {
                        "type": "boolean",
                        "default": True,
                    },
                    "kernel-device-trees": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-modules": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-configured-modules": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-stage-firmware": {
                        "type": "boolean",
                        "default": False,
                    },
                    "kernel-initrd-firmware": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-compression": {
                        "type": "string",
                        "enum": ["lz4", "xz", "gz", "zstd"],
                    },
                    "kernel-initrd-compression-options": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-initrd-overlay": {
                        "type": "string",
                        "default": "",
                    },
                    "kernel-initrd-addons": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-compiler": {
                        "type": "string",
                        "default": "",
                    },
                    "kernel-compiler-paths": {
                        "type": "array",
                        "minitems": 1,
                        "uniqueItems": True,
                        "items": {"type": "string"},
                        "default": [],
                    },
                    "kernel-compiler-parameters": {
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
                    "kernel-add-ppa": {
                        "type": "boolean",
                        "default": True,
                    },
                    "kernel-use-llvm": {
                        "oneOf": [{"type": "boolean"}, {"type": "string"}],
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
                "gcc",
                "cmake",
                "cryptsetup",
                "dracut-core",
                "kmod",
                "kpartx",
                "lz4",
                "systemd",
            },
        )

    def test_get_base_build_packages_lz4(self):
        plugin = self._setup_test(kernelinitrdcompression="lz4")
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "bc",
                "binutils",
                "gcc",
                "cmake",
                "cryptsetup",
                "dracut-core",
                "kmod",
                "kpartx",
                "lz4",
                "systemd",
            },
        )

    def test_get_base_build_packages_xz(self):
        plugin = self._setup_test(kernelinitrdcompression="xz")
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "bc",
                "binutils",
                "gcc",
                "cmake",
                "cryptsetup",
                "dracut-core",
                "kmod",
                "kpartx",
                "xz-utils",
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
                "gcc",
                "cmake",
                "cryptsetup",
                "dracut-core",
                "kmod",
                "kpartx",
                "lz4",
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
                "gcc",
                "cmake",
                "cryptsetup",
                "dracut-core",
                "kmod",
                "kpartx",
                "lz4",
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
        self.assertIs(opt.kernel_kconfigfile, None)
        self.assertEqual(opt.kernel_kconfigflavour, None)
        self.assertEqual(opt.kernel_kconfigs, None)
        self.assertEqual(opt.kernel_image_target, "Image.gz")
        self.assertTrue(opt.kernel_with_firmware)
        self.assertEqual(opt.kernel_device_trees, None)
        self.assertEqual(opt.kernel_compiler, None)
        self.assertEqual(opt.kernel_compiler_paths, None)
        self.assertEqual(opt.kernel_compiler_parameters, None)
        self.assertEqual(opt.kernel_initrd_modules, None)
        self.assertEqual(opt.kernel_initrd_configured_modules, None)
        self.assertEqual(opt.kernel_initrd_firmware, None)
        self.assertEqual(opt.kernel_initrd_compression, None)
        self.assertEqual(opt.kernel_initrd_compression_options, None)
        self.assertEqual(opt.kernel_initrd_overlay, None)
        self.assertEqual(opt.kernel_initrd_addons, None)
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)
        self.assertFalse(opt.kernel_add_ppa)
        self.assertFalse(opt.kernel_use_llvm)

    def test_check_configuration_kde_config(self):
        plugin = self._setup_test(
            kernelkdefconfig=["snappy_defconfig"],
            kernelimagetarget="Image",
            kernelwithfirmware=True,
        )
        opt = plugin.options

        self.assertEqual(opt.kernel_kdefconfig, ["snappy_defconfig"])
        self.assertIs(opt.kernel_kconfigfile, None)
        self.assertIs(opt.kernel_kconfigflavour, None)
        self.assertIs(opt.kernel_kconfigs, None)
        self.assertEqual(opt.kernel_image_target, "Image")
        self.assertTrue(opt.kernel_with_firmware)
        self.assertIs(opt.kernel_device_trees, None)
        self.assertIs(opt.kernel_compiler, None)
        self.assertIs(opt.kernel_compiler_paths, None)
        self.assertIs(opt.kernel_compiler_parameters, None)
        self.assertIs(opt.kernel_initrd_modules, None)
        self.assertIs(opt.kernel_initrd_configured_modules, None)
        self.assertIs(opt.kernel_initrd_firmware, None)
        self.assertIs(opt.kernel_initrd_compression, None)
        self.assertIs(opt.kernel_initrd_compression_options, None)
        self.assertIs(opt.kernel_initrd_overlay, None)
        self.assertIs(opt.kernel_initrd_addons, None)
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)
        self.assertFalse(opt.kernel_add_ppa)
        self.assertFalse(opt.kernel_use_llvm)

    def test_check_configuration_image_target(self):
        plugin = self._setup_test(
            kernelkdefconfig=["snappy_defconfig"],
            kernelimagetarget={"arm64": "Image", "armhf": "Image.gz"},
            kernelwithfirmware=True,
            kernelinitrdcompression="zstd",
        )
        opt = plugin.options

        self.assertEqual(opt.kernel_kdefconfig, ["snappy_defconfig"])
        self.assertIs(opt.kernel_kconfigfile, None)
        self.assertIs(opt.kernel_kconfigflavour, None)
        self.assertIs(opt.kernel_kconfigs, None)
        self.assertEqual(
            opt.kernel_image_target, {"arm64": "Image", "armhf": "Image.gz"}
        )
        self.assertTrue(opt.kernel_with_firmware)
        self.assertIs(opt.kernel_device_trees, None)
        self.assertIs(opt.kernel_compiler, None)
        self.assertIs(opt.kernel_compiler_paths, None)
        self.assertIs(opt.kernel_compiler_parameters, None)
        self.assertIs(opt.kernel_initrd_modules, None)
        self.assertIs(opt.kernel_initrd_configured_modules, None)
        self.assertIs(opt.kernel_initrd_firmware, None)
        self.assertEqual(opt.kernel_initrd_compression, "zstd")
        self.assertIs(opt.kernel_initrd_compression_options, None)
        self.assertIs(opt.kernel_initrd_overlay, None)
        self.assertIs(opt.kernel_initrd_addons, None)
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)
        self.assertFalse(opt.kernel_add_ppa)
        self.assertFalse(opt.kernel_use_llvm)

    def test_check_configuration_konfig_file(self):
        plugin = self._setup_test(
            kernelkconfigfile="arch/arm64/configs/snappy_defconfig",
            kernelwithfirmware=True,
            kernelkconfigs=["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
            kernelinitrdcompression="lz4",
            kernelinitrdcompressionoptions=["-9", "-l"],
        )
        opt = plugin.options

        self.assertEqual(opt.kernel_kdefconfig, ["defconfig"])
        self.assertEqual(opt.kernel_kconfigfile, "arch/arm64/configs/snappy_defconfig")
        self.assertIs(opt.kernel_kconfigflavour, None)
        self.assertEqual(
            opt.kernel_kconfigs, ["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"]
        )
        self.assertEqual(opt.kernel_image_target, "Image.gz")
        self.assertTrue(opt.kernel_with_firmware)
        self.assertIs(opt.kernel_device_trees, None)
        self.assertIs(opt.kernel_compiler, None)
        self.assertIs(opt.kernel_compiler_paths, None)
        self.assertIs(opt.kernel_compiler_parameters, None)
        self.assertIs(opt.kernel_initrd_modules, None)
        self.assertIs(opt.kernel_initrd_configured_modules, None)
        self.assertIs(opt.kernel_initrd_firmware, None)
        self.assertEqual(opt.kernel_initrd_compression, "lz4")
        self.assertEqual(opt.kernel_initrd_compression_options, ["-9", "-l"])
        self.assertIs(opt.kernel_initrd_overlay, None)
        self.assertIs(opt.kernel_initrd_addons, None)
        self.assertFalse(opt.kernel_enable_zfs_support)
        self.assertFalse(opt.kernel_enable_perf)
        self.assertFalse(opt.kernel_add_ppa)
        self.assertFalse(opt.kernel_use_llvm)

    def test_check_get_build_environment(self):
        plugin = self._setup_test()
        plugin.kernel_arch = "amd64"

        self.assertEqual(
            plugin.get_build_environment(),
            {
                "CROSS_COMPILE": "${SNAPCRAFT_ARCH_TRIPLET}-",
                "ARCH": plugin.kernel_arch,
                "DEB_ARCH": "${SNAPCRAFT_TARGET_ARCH}",
                "UC_INITRD_DEB": "${SNAPCRAFT_PART_BUILD}/ubuntu-core-initramfs",
                "SNAPD_UNPACKED_SNAP": "${SNAPCRAFT_PART_BUILD}/unpacked_snapd",
                "KERNEL_BUILD_ARCH_DIR": f"${{SNAPCRAFT_PART_BUILD}}/arch/{plugin.kernel_arch}/boot",
                "KERNEL_IMAGE_TARGET": plugin.kernel_image_target,
            },
        )

    def test_check_get_build_environment_compiler_paths_cross(self):
        plugin = self._setup_test(
            kernelcompilerpaths=["gcc-11/bin"],
            kernelimagetarget={"arm64": "Image", "armhf": "Image.gz"},
            arch="armv7l",
        )

        plugin._set_kernel_targets()
        self.assertEqual(
            plugin.get_build_environment(),
            {
                "CROSS_COMPILE": "${SNAPCRAFT_ARCH_TRIPLET}-",
                "ARCH": "arm",
                "DEB_ARCH": "${SNAPCRAFT_TARGET_ARCH}",
                "UC_INITRD_DEB": "${SNAPCRAFT_PART_BUILD}/ubuntu-core-initramfs",
                "SNAPD_UNPACKED_SNAP": "${SNAPCRAFT_PART_BUILD}/unpacked_snapd",
                "KERNEL_BUILD_ARCH_DIR": "${SNAPCRAFT_PART_BUILD}/arch/arm/boot",
                "KERNEL_IMAGE_TARGET": "Image.gz",
                "PATH": "${SNAPCRAFT_STAGE}/gcc-11/bin:${PATH}",
            },
        )

    def test_check_get_build_environment_compiler_paths(self):
        plugin = self._setup_test(
            kernelcompilerpaths=["gcc-11/bin", "gcc-11/sbin"],
        )
        plugin.kernel_arch = "amd64"

        self.assertEqual(
            plugin.get_build_environment(),
            {
                "CROSS_COMPILE": "${SNAPCRAFT_ARCH_TRIPLET}-",
                "ARCH": plugin.kernel_arch,
                "DEB_ARCH": "${SNAPCRAFT_TARGET_ARCH}",
                "UC_INITRD_DEB": "${SNAPCRAFT_PART_BUILD}/ubuntu-core-initramfs",
                "SNAPD_UNPACKED_SNAP": "${SNAPCRAFT_PART_BUILD}/unpacked_snapd",
                "KERNEL_BUILD_ARCH_DIR": f"${{SNAPCRAFT_PART_BUILD}}/arch/{plugin.kernel_arch}/boot",
                "KERNEL_IMAGE_TARGET": plugin.kernel_image_target,
                "PATH": "${SNAPCRAFT_STAGE}/gcc-11/bin:${SNAPCRAFT_STAGE}/gcc-11/sbin:${PATH}",
            },
        )

    def test_check_get_build_command(self):
        plugin = self._setup_test()

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert _is_sub_array(build_commands, _initrd_modules_empty_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _donwload_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
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
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _configure_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert not _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert _is_sub_array(build_commands, _initrd_check_firmware_part)
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert not _is_sub_array(build_commands, _build_zfs_cmd)
        assert not _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_defconfig_configs_no_firmware_lz4(self):
        plugin = self._setup_test(
            kernelkconfigfile="arch/arm64/configs/snappy_defconfig",
            kernelcompiler="clang",
            kernelcompilerparameters=["-arch", "arm64"],
            kernelimagetarget="Image",
            kernelwithfirmware=False,
            kernelkconfigs=["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
            kernelinitrdcompression="lz4",
            kernelinitrdcompressionoptions=["-9", "-l"],
            kernelinitrdmodules=["dm-crypt", "slimbus"],
            kernelinitrdstagefirmware=True,
            kernelinitrdfirmware=["firmware/for/wifi", "firmware/for/webcam"],
            kernelinitrdaddons=[
                "usr/bin/cryptsetup",
                "usr/lib/my-arch/libcrypto.so",
            ],
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert _is_sub_array(build_commands, _initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _donwload_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
        assert not _is_sub_array(build_commands, _clone_zfs_cmd)
        assert _is_sub_array(build_commands, _clean_old_build_cmd)
        assert _is_sub_array(build_commands, _prepare_config_defconfig_cmd)
        assert _is_sub_array(build_commands, _prepare_config_extra_config_cmd)
        assert _is_sub_array(build_commands, _remake_old_config_clang_cmd)
        assert _check_config in build_commands
        if platform.machine() == "x86_64":
            assert _is_sub_array(build_commands, _build_kernel_clang_image_x86_cmd)
            assert _is_sub_array(
                build_commands, _install_kernel_no_firmware_clang_x86_cmd
            )
        else:
            assert _is_sub_array(build_commands, _build_kernel_clang_image_cmd)
            assert _is_sub_array(build_commands, _install_kernel_no_firmware_clang_cmd)

        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _configure_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert not _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert _is_sub_array(build_commands, _initrd_check_firmware_stage)
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_stage_firmware_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert not _is_sub_array(build_commands, _build_zfs_cmd)
        assert not _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_unknown_compiler(self):
        plugin = self._setup_test(
            kernelcompiler="my-gcc",
        )

        # we need to get build environment
        plugin.get_build_environment()
        with self.assertLogs(level=logging.INFO) as cm:
            build_commands = plugin.get_build_commands()

        # there is "INFO:snapcraft_legacy.plugins.v2.kernel:Getting build commands..."
        assert len(cm.output) == 3
        assert "Only other 'supported' compiler is clang" in cm.output[1]
        assert "hopefully you know what you are doing" in cm.output[2]
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert _is_sub_array(build_commands, _initrd_modules_empty_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _donwload_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
        assert not _is_sub_array(build_commands, _clone_zfs_cmd)
        assert _is_sub_array(build_commands, _clean_old_build_cmd)
        assert _is_sub_array(build_commands, _prepare_config_custom_cc_cmd)
        assert _is_sub_array(build_commands, _remake_old_config_custom_cc_cmd)
        assert _check_config in build_commands
        if platform.machine() == "x86_64":
            assert _is_sub_array(build_commands, _build_kernel_x86_custom_cc_cmd)
            assert _is_sub_array(build_commands, _install_kernel_x86_custom_cc_cmd)
        else:
            assert _is_sub_array(build_commands, _build_kernel_custom_cc_cmd)
            assert _is_sub_array(build_commands, _install_kernel_custom_cc_cmd)

        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _configure_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert not _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert _is_sub_array(build_commands, _initrd_check_firmware_part)
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert not _is_sub_array(build_commands, _build_zfs_cmd)
        assert not _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_config_flavour_configs(self):
        plugin = self._setup_test(
            kernelkconfigflavour="raspi",
            kernelkconfigs=["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
            kerneldevicetrees=["pi3", "pi3b", "pi4", "pi/cm3", "pi/cm4"],
            kernelwithfirmware=False,
            kernelenablezfssupport=True,
            kernelenableperf=True,
            kernelinitrdmodules=["dm-crypt", "slimbus"],
            kernelinitrdconfiguredmodules=["libarc4"],
            kernelinitrdoverlay="my-overlay",
            kernelinitrdcompression="gz",
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert _is_sub_array(build_commands, _initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _donwload_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
        assert _is_sub_array(build_commands, _clone_zfs_cmd)
        assert _is_sub_array(build_commands, _clean_old_build_cmd)
        assert _is_sub_array(build_commands, _prepare_config_flavour_cmd)
        assert _is_sub_array(build_commands, _prepare_config_extra_config_cmd)
        assert _is_sub_array(build_commands, _remake_old_config_cmd)
        assert _check_config in build_commands
        assert _is_sub_array(build_commands, _build_kernel_dtbs_cmd)
        assert _is_sub_array(build_commands, _install_kernel_no_dtbs_no_firmware_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_dtbs_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _configure_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert _is_sub_array(build_commands, _initrd_check_firmware_part)
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert _is_sub_array(build_commands, _update_initrd_compression_gz_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
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
            kernelinitrdmodules=["dm-crypt", "slimbus"],
            kernelinitrdconfiguredmodules=["libarc4"],
            kernelinitrdoverlay="my-overlay",
            arch="armv7l",
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
        assert _is_sub_array(build_commands, _initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _donwload_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_armhf_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_armhf_cmd)
        assert _is_sub_array(build_commands, _clone_zfs_cmd)
        assert _is_sub_array(build_commands, _clean_old_build_cmd)
        assert _is_sub_array(build_commands, _prepare_config_flavour_cmd)
        assert _is_sub_array(build_commands, _prepare_config_extra_config_cmd)
        assert _is_sub_array(build_commands, _remake_old_config_armhf_cmd)
        assert _check_config in build_commands
        assert _is_sub_array(build_commands, _build_kernel_armhf_cmd)
        assert _is_sub_array(build_commands, _install_kernel_armhf_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert not _is_sub_array(build_commands, _install_dtbs_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _configure_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert _is_sub_array(build_commands, _initrd_check_firmware_part)
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
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
        plugin.target_arch = _DEB_ARCH_TRANSLATIONS[arch]
        plugin.kernel_arch = None
        plugin.deb_arch = None
        plugin._get_deb_architecture()
        plugin._get_kernel_architecture()
        assert plugin.kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin.deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_armhf(self):
        arch = "armv7l"
        cross_building = False
        if platform.machine() != arch:
            cross_building = True
        plugin = self._setup_test(arch=arch)
        plugin.target_arch = _DEB_ARCH_TRANSLATIONS[arch]
        plugin.kernel_arch = None
        plugin.deb_arch = None
        plugin._get_deb_architecture()
        plugin._get_kernel_architecture()
        assert plugin.kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin.deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_riscv64(self):
        arch = "riscv64"
        cross_building = False
        if platform.machine() != arch:
            cross_building = True
        plugin = self._setup_test(arch=arch)
        plugin.target_arch = _DEB_ARCH_TRANSLATIONS[arch]
        plugin.kernel_arch = None
        plugin.deb_arch = None
        plugin._get_deb_architecture()
        plugin._get_kernel_architecture()
        assert plugin.kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin.deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_x86_64(self):
        arch = "x86_64"
        cross_building = False
        if platform.machine() != arch:
            cross_building = True
        plugin = self._setup_test(arch=arch)
        plugin.target_arch = _DEB_ARCH_TRANSLATIONS[arch]
        plugin.kernel_arch = None
        plugin.deb_arch = None
        plugin._get_deb_architecture()
        plugin._get_kernel_architecture()
        assert plugin.kernel_arch == _KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin.deb_arch == _DEB_ARCH_TRANSLATIONS[arch]
        assert plugin._cross_building == cross_building

    def test_check_arch_i686(self):
        # we do not support i686 so use this arch to test unknnown arch by plugin
        arch = "i686"
        plugin = self._setup_test(arch=arch)
        plugin.target_arch = _DEB_ARCH_TRANSLATIONS[arch]
        plugin.kernel_arch = None
        plugin.deb_arch = None
        plugin._get_deb_architecture()
        plugin._get_kernel_architecture()
        assert plugin.kernel_arch is None
        assert plugin.deb_arch is None

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
                check_new_config(config_path=config_file.name, initrd_modules=[])
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
                check_new_config(config_path=config_file.name, initrd_modules=[])
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
            assert (
                "adding\nthe corresponding module to initrd:\n\nCONFIG_SQUASHFS\n"
                in cm.output[1]
            )
            assert "CONFIG_SQUASHFS" in cm.output[1]

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
                check_new_config(config_path=config_file.name, initrd_modules=[])
            # there should be 1 warning log for missing module in initrd
            assert len(cm.output) == 1
            assert (
                "**** WARNING **** WARNING **** WARNING **** WARNING ****"
                in cm.output[0]
            )
            assert (
                "adding\nthe corresponding module to initrd:\n\nCONFIG_SQUASHFS\n"
                in cm.output[0]
            )
            assert "CONFIG_SQUASHFS" in cm.output[0]

    def test_check_new_config_squash_module(self):
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
                # assert log does not support no log case, we log and check that
                # if the only log
                logging.getLogger("kernel.py").warning("ONLY WARNING")
                check_new_config(
                    config_path=config_file.name, initrd_modules=["squashfs"]
                )
            self.assertEqual(cm.output, ["WARNING:kernel.py:ONLY WARNING"])

    def test_check_new_config_squash_missing_file(self):
        # run with invalid file
        e = ""
        try:
            check_new_config(config_path="wrong/file", initrd_modules=["suashfs"])
        except FileNotFoundError as err:
            e = err.strerror
        assert e == "No such file or directory"

    def test_use_llvm(self):
        plugin = self._setup_test(kernelusellvm=True)
        self.assertEqual("1", plugin._llvm_version)
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "bc",
                "binutils",
                "gcc",
                "cmake",
                "cryptsetup",
                "dracut-core",
                "kmod",
                "kpartx",
                "lld",
                "llvm",
                "lz4",
                "systemd",
            },
        )
        plugin.get_build_environment()
        cmd = plugin.get_build_commands()
        targets = f"{plugin.kernel_image_target} modules"
        if plugin.kernel_arch in ("arm", "arm64", "riscv64"):
            targets += " dtbs"
        self.assertIn(
            f'make -j$(nproc) -C ${{KERNEL_SRC}} O=${{SNAPCRAFT_PART_BUILD}} LLVM="1" {targets}',
            cmd,
        )

    def test_use_llvm_specific_version(self):
        version = "-10"
        plugin = self._setup_test(kernelusellvm=version)
        self.assertEqual(version, plugin._llvm_version)
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "bc",
                "binutils",
                "gcc",
                "cmake",
                "cryptsetup",
                "dracut-core",
                "kmod",
                "kpartx",
                f"lld{version}",
                f"llvm{version}",
                "lz4",
                "systemd",
            },
        )
        plugin.get_build_environment()
        cmd = plugin.get_build_commands()
        targets = f"{plugin.kernel_image_target} modules"
        if plugin.kernel_arch in ("arm", "arm64", "riscv64"):
            targets += " dtbs"
        self.assertIn(
            f'make -j$(nproc) -C ${{KERNEL_SRC}} O=${{SNAPCRAFT_PART_BUILD}} LLVM="{version}" {targets}',
            cmd,
        )

    def test_bad_llvm_version(self):
        self.assertRaises(ValueError, self._setup_test, kernelusellvm="-badsuffix")


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

_initrd_modules_empty_cmd = ['initrd_installed_kernel_modules=""']

_initrd_modules_cmd = ['initrd_installed_kernel_modules="dm-crypt slimbus"']

_initrd_configured_modules_empty_cmd = ['initrd_configured_kernel_modules=""']

_initrd_configured_modules_cmd = ['initrd_configured_kernel_modules="libarc4"']

_link_files_fnc = [
    "link_files() {",
    '\tif [ "${2}" = "*" ]; then',
    "\t\tfor f in $(ls ${1})",
    "\t\tdo",
    '\t\t\tlink_files "${1}" "${f}" "${3}"',
    "\t\tdone",
    "\t\treturn 0",
    "\tfi",
    '\tif [ -d "${1}/${2}" ]; then',
    "\t\tfor f in $(ls ${1}/${2})",
    "\t\tdo",
    '\t\t\tlink_files "${1}" "${2}/${f}" "${3}"',
    "\t\tdone",
    "\t\treturn 0",
    "\tfi",
    "",
    '\tlocal found=""',
    "\tfor f in $(ls ${1}/${2})",
    "\tdo",
    '\t\tif [[ -L "${f}" ]]; then',
    "\t\t\tlocal rel_path=$( realpath --no-symlinks --relative-to=${1} ${f} )",
    "\t\telse",
    "\t\t\tlocal rel_path=$( realpath -se --relative-to=${1} ${f} )",
    "\t\tfi",
    "\t\tlocal dir_path=$(dirname ${rel_path})",
    "\t\tmkdir -p ${3}/${dir_path}",
    '\t\techo "installing ${f} to ${3}/${dir_path}"',
    "\t\tln -f ${f} ${3}/${dir_path}",
    '\t\tfound="yes"',
    "\tdone",
    '\tif [ "yes" = "${found}" ]; then',
    "\t\treturn 0",
    "\telse",
    "\t\treturn 1",
    "\tfi",
    "}",
]

_donwload_initrd_fnc = [
    "download_core_initrd() {",
    "\tapt-get download ubuntu-core-initramfs:${1}",
    "\t# unpack dep to the target dir",
    "\tdpkg -x ubuntu-core-initramfs_*.deb ${2}",
    "}",
]

_get_initrd_cmd = [
    'echo "Getting ubuntu-core-initrd...."',
    "if [ ! -e ${UC_INITRD_DEB} ]; then",
    f"\tdownload_core_initrd {_DEB_ARCH_TRANSLATIONS[platform.machine()]} ${{UC_INITRD_DEB}}",
    "fi",
]

_get_initrd_armhf_cmd = [
    'echo "Getting ubuntu-core-initrd...."',
    "if [ ! -e ${UC_INITRD_DEB} ]; then",
    "\tdownload_core_initrd armhf ${UC_INITRD_DEB}",
    "fi",
]

_download_snapd_fnc = [
    "\tapt-get download snapd:${1}",
    "\t# unpack dep to the target dir",
    "\tdpkg -x snapd_*.deb ${2}",
    "}",
]

_get_snapd_cmd = [
    'echo "Getting snapd deb for snap bootstrap..."',
    "if [ ! -e ${SNAPD_UNPACKED_SNAP} ]; then",
    f"\tdownload_snap_bootstrap {_DEB_ARCH_TRANSLATIONS[platform.machine()]} ${{SNAPD_UNPACKED_SNAP}}",
    "fi",
]

_get_snapd_armhf_cmd = [
    'echo "Getting snapd deb for snap bootstrap..."',
    "if [ ! -e ${SNAPD_UNPACKED_SNAP} ]; then",
    "\tdownload_snap_bootstrap armhf ${SNAPD_UNPACKED_SNAP}",
    "fi",
]

_clean_old_build_cmd = [
    'echo "Cleaning previous build first..."',
    "[ -e ${SNAPCRAFT_PART_INSTALL}/modules ] && rm -rf ${SNAPCRAFT_PART_INSTALL}/modules",
    "[ -L ${SNAPCRAFT_PART_INSTALL}/lib/modules ] && rm -rf ${SNAPCRAFT_PART_INSTALL}/lib/modules",
]

_prepare_config_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${SNAPCRAFT_PART_BUILD}/.config ]; then",
    "\t make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} defconfig",
    "fi",
]

_prepare_config_custom_cc_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${SNAPCRAFT_PART_BUILD}/.config ]; then",
    '\t make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} CC="my-gcc" defconfig',
    "fi",
]

_prepare_config_defconfig_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${SNAPCRAFT_PART_BUILD}/.config ]; then",
    "\t cp arch/arm64/configs/snappy_defconfig ${SNAPCRAFT_PART_BUILD}/.config",
    "fi",
]

_prepare_config_flavour_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${SNAPCRAFT_PART_BUILD}/.config ]; then",
    '\techo "Assembling Ubuntu config..."',
    "\tbranch=$(cut -d'.' -f 2- < ${KERNEL_SRC}/debian/debian.env)",
    "\tbaseconfigdir=${KERNEL_SRC}/debian.${branch}/config",
    "\tarchconfigdir=${KERNEL_SRC}/debian.${branch}/config/${DEB_ARCH}",
    "\tcommonconfig=${baseconfigdir}/config.common.ports",
    "\tubuntuconfig=${baseconfigdir}/config.common.ubuntu",
    "\tarchconfig=${archconfigdir}/config.common.${DEB_ARCH}",
    "\tflavourconfig=${archconfigdir}/config.flavour.raspi",
    " ".join(
        [
            "\tcat",
            "${commonconfig} ${ubuntuconfig} ${archconfig} ${flavourconfig}",
            "> ${SNAPCRAFT_PART_BUILD}/.config 2>/dev/null",
        ],
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
    "bash -c ' yes \"\" || true' | make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} oldconfig",
]

_remake_old_config_custom_cc_cmd = [
    'echo "Remaking oldconfig...."',
    "bash -c ' yes \"\" || true'"
    ' | make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} CC="my-gcc" oldconfig',
]

_remake_old_config_clang_cmd = [
    'echo "Remaking oldconfig...."',
    " ".join(
        [
            "bash -c ' yes \"\" || true' |",
            "make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD}",
            'CC="clang"',
            "-arch arm64",
            "oldconfig",
        ],
    ),
]

_remake_old_config_armhf_cmd = [
    'echo "Remaking oldconfig...."',
    " ".join(
        [
            "bash -c ' yes \"\" || true' |",
            "make -j1 -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD}",
            "ARCH=arm CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET}-",
            "oldconfig",
        ],
    ),
]

_check_config = " ".join(
    [
        sys.executable,
        "-I",
        inspect.getfile(KernelPlugin),
        "check_new_config",
        "${SNAPCRAFT_PART_BUILD}/.config",
        "${initrd_installed_kernel_modules}",
        "${initrd_configured_kernel_modules}",
        "",
    ],
)

_build_kernel_cmd = [
    "make -j$(nproc) -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} Image.gz modules dtbs",
]

_build_kernel_x86_cmd = [
    "make -j$(nproc) -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} Image.gz modules",
]

_build_kernel_custom_cc_cmd = [
    'make -j$(nproc) -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} CC="my-gcc" Image.gz modules dtbs',
]

_build_kernel_x86_custom_cc_cmd = [
    'make -j$(nproc) -C ${KERNEL_SRC} O=${SNAPCRAFT_PART_BUILD} CC="my-gcc" Image.gz modules',
]

_build_kernel_clang_image_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            'CC="clang"',
            "-arch arm64",
            "Image modules dtbs",
        ],
    ),
]

_build_kernel_clang_image_x86_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            'CC="clang"',
            "-arch arm64",
            "Image modules",
        ],
    ),
]

_build_kernel_armhf_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "ARCH=arm",
            "CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET}-",
            "Image.gz modules dtbs",
        ],
    ),
]

_build_kernel_dtbs_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "Image.gz modules pi3.dtb pi3b.dtb pi4.dtb pi/cm3.dtb pi/cm4.dtb",
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
            "firmware_install INSTALL_FW_PATH=${SNAPCRAFT_PART_INSTALL}/lib/firmware",
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
            "firmware_install INSTALL_FW_PATH=${SNAPCRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_install_kernel_custom_cc_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            'CC="my-gcc"',
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${SNAPCRAFT_PART_INSTALL}/dtbs",
            "firmware_install INSTALL_FW_PATH=${SNAPCRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_install_kernel_x86_custom_cc_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            'CC="my-gcc"',
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
            "firmware_install INSTALL_FW_PATH=${SNAPCRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_install_kernel_no_dtbs_no_firmware_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
        ],
    ),
]

_install_kernel_no_firmware_clang_cmd = [
    " ".join(
        [
            "make",
            "-j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            'CC="clang"',
            "-arch arm64",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${SNAPCRAFT_PART_INSTALL}/dtbs",
        ],
    ),
]

_install_kernel_no_firmware_clang_x86_cmd = [
    " ".join(
        [
            "make",
            "-j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            'CC="clang"',
            "-arch arm64",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
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
            "CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET}-",
            "CONFIG_PREFIX=${SNAPCRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${SNAPCRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${SNAPCRAFT_PART_INSTALL}/dtbs",
            "firmware_install INSTALL_FW_PATH=${SNAPCRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_parse_kernel_release_cmd = [
    "KERNEL_RELEASE=$(cat ${SNAPCRAFT_PART_BUILD}/include/config/kernel.release)",
]

_intall_kernel_cmd = [
    "[ -e ${SNAPCRAFT_PART_INSTALL}/kernel.img ] && rm -rf ${SNAPCRAFT_PART_INSTALL}/kernel.img",
    " ".join(
        [
            "ln -f",
            "${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET}",
            "${SNAPCRAFT_PART_INSTALL}/${KERNEL_IMAGE_TARGET}-${KERNEL_RELEASE}",
        ],
    ),
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET} ${SNAPCRAFT_PART_INSTALL}/kernel.img",
    "",
    'echo "Copying System map..."',
    "[ -e ${SNAPCRAFT_PART_INSTALL}/System.map ] && rm -rf ${SNAPCRAFT_PART_INSTALL}/System.map*",
    "ln -f ${SNAPCRAFT_PART_BUILD}/System.map ${SNAPCRAFT_PART_INSTALL}/System.map-${KERNEL_RELEASE}",
]

_install_dtbs_cmd = [
    'echo "Copying custom dtbs..."',
    "mkdir -p ${SNAPCRAFT_PART_INSTALL}/dtbs",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi3.dtb ${SNAPCRAFT_PART_INSTALL}/dtbs/pi3.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi3b.dtb ${SNAPCRAFT_PART_INSTALL}/dtbs/pi3b.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi4.dtb ${SNAPCRAFT_PART_INSTALL}/dtbs/pi4.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi/cm3.dtb ${SNAPCRAFT_PART_INSTALL}/dtbs/cm3.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi/cm4.dtb ${SNAPCRAFT_PART_INSTALL}/dtbs/cm4.dtb",
]

_install_initrd_modules_cmd = [
    'echo "Installing ko modules to initrd..."',
    'install_modules=""',
    'echo "Gathering module dependencies..."',
    'install_modules=""',
    "uc_initrd_feature_kernel_modules=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/kernel-modules",
    "mkdir -p ${uc_initrd_feature_kernel_modules}",
    "initramfs_ko_modules_conf=${uc_initrd_feature_kernel_modules}/extra-kernel-modules.conf",
    "for m in ${initrd_installed_kernel_modules} ${initrd_configured_kernel_modules}",
    "do",
    "\techo ${m} >> ${initramfs_ko_modules_conf}",
    "done",
]

_configure_initrd_modules_cmd = [
    " ".join(
        [
            "[ -e ${initramfs_ko_modules_conf} ]",
            "&&",
            "sort -fu ${initramfs_ko_modules_conf} -o ${initramfs_ko_modules_conf}",
        ],
    ),
    'echo "Configuring ubuntu-core-initramfs.conf with supported modules"',
    'echo "If module is not included in initrd, do not include it"',
    "initramfs_conf_dir=${uc_initrd_feature_kernel_modules}/usr/lib/modules-load.d",
    "mkdir -p ${initramfs_conf_dir}",
    "initramfs_conf=${initramfs_conf_dir}/ubuntu-core-initramfs.conf",
    'echo "# configures modules" > ${initramfs_conf}',
    "for m in ${initrd_configured_kernel_modules}",
    "do",
    " ".join(
        [
            "\tif",
            "[ -n",
            '"$(modprobe -n -q --show-depends',
            "-d ${uc_initrd_feature_kernel_modules}",
            '-S "${KERNEL_RELEASE}"',
            '${m})" ]; then',
        ],
    ),
    "\t\techo ${m} >> ${initramfs_conf}",
    "\tfi",
    "done",
]

_initrd_overlay_features_cmd = [
    "uc_initrd_feature_firmware=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/uc-firmware",
    "mkdir -p ${uc_initrd_feature_firmware}",
    "uc_initrd_feature_overlay=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/uc-overlay",
    "mkdir -p ${uc_initrd_feature_overlay}",
]

_install_initrd_firmware_cmd = [
    'echo "Installing initrd overlay firmware..."',
    "for f in firmware/for/wifi firmware/for/webcam",
    "do",
    '\tif ! link_files "${SNAPCRAFT_PART_INSTALL}" "${f}" "${uc_initrd_feature_firmware}/lib" ; then',
    '\t\tif ! link_files "${SNAPCRAFT_STAGE}" "${f}" "${uc_initrd_feature_firmware}/lib" ; then',
    '\t\t\techo "Missing firmware [${f}], ignoring it"',
    "\t\tfi",
    "\tfi",
    "done",
]

_install_initrd_addons_cmd = [
    'echo "Installing initrd addons..."',
    "for a in usr/bin/cryptsetup usr/lib/my-arch/libcrypto.so",
    "do",
    '\techo "Copy overlay: ${a}"',
    '\tlink_files "${SNAPCRAFT_STAGE}" "${a}" "${uc_initrd_feature_overlay}"',
    "done",
]

_intatll_initrd_overlay_cmd = [
    'link_files "${SNAPCRAFT_STAGE}/my-overlay" "" "${uc_initrd_feature_overlay}"'
]

_prepare_ininird_features_cmd = [
    'echo "Preparing snap-boostrap initrd feature..."',
    "uc_initrd_feature_snap_bootstratp=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snap-bootstrap",
    "mkdir -p ${uc_initrd_feature_snap_bootstratp}",
    " ".join(
        [
            "link_files",
            '"${SNAPD_UNPACKED_SNAP}" "usr/lib/snapd/snap-bootstrap"',
            '"${uc_initrd_feature_snap_bootstratp}"',
        ],
    ),
    'link_files "${SNAPD_UNPACKED_SNAP}" "usr/lib/snapd/info" "${uc_initrd_feature_snap_bootstratp}"',
    "cp ${SNAPD_UNPACKED_SNAP}/usr/lib/snapd/info ${SNAPCRAFT_PART_INSTALL}/snapd-info",
]

_clean_old_initrd_cmd = [
    "if compgen -G  ${SNAPCRAFT_PART_INSTALL}/initrd.img* >  /dev/null; then",
    "\trm -rf ${SNAPCRAFT_PART_INSTALL}/initrd.img*",
    "fi",
]

_initrd_check_firmware_stage = [
    '[ ! -d "${SNAPCRAFT_STAGE}/firmware" ] && echo -e "firmware directory '
    "${SNAPCRAFT_STAGE}/firmware does not exist, consider using "
    'kernel-initrd-stage-firmware: true/false option" && exit 1'
]

_initrd_check_firmware_part = [
    '[ ! -d "${SNAPCRAFT_PART_INSTALL}/lib/firmware" ] && echo -e "firmware directory '
    "${SNAPCRAFT_PART_INSTALL}/lib/firmware does not exist, consider using "
    'kernel-initrd-stage-firmware: true/false option" && exit 1'
]

_initrd_tool_cmd = [
    "ubuntu_core_initramfs=${UC_INITRD_DEB}/usr/bin/ubuntu-core-initramfs",
]

_update_initrd_compression_cmd = [
    'echo "Updating compression command to be used for initrd"',
    "sed -i 's/lz4 -9 -l/lz4 -9 -l/g' ${ubuntu_core_initramfs}",
]

_update_initrd_compression_gz_cmd = [
    'echo "Updating compression command to be used for initrd"',
    "sed -i 's/lz4 -9 -l/gzip -7/g' ${ubuntu_core_initramfs}",
]

_initrd_tool_workaroud_cmd = [
    "for feature in kernel-modules snap-bootstrap uc-firmware uc-overlay",
    "do",
    " ".join(
        [
            '\tlink_files "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/${feature}"',
            '"*"',
            '"${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/main"',
        ],
    ),
    "done",
]

_create_inird_cmd = [
    " ".join(
        [
            "${ubuntu_core_initramfs}",
            "create-initrd",
            "--root ${UC_INITRD_DEB}",
            "--kernelver=${KERNEL_RELEASE}",
            "--kerneldir ${SNAPCRAFT_PART_INSTALL}/lib/modules/${KERNEL_RELEASE}",
            "--firmwaredir ${SNAPCRAFT_PART_INSTALL}/lib/firmware",
            "--skeleton ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs",
            "--output ${SNAPCRAFT_PART_INSTALL}/initrd.img",
        ],
    ),
    "ln $(ls ${SNAPCRAFT_PART_INSTALL}/initrd.img*) ${SNAPCRAFT_PART_INSTALL}/initrd.img",
]

_create_inird_stage_firmware_cmd = [
    " ".join(
        [
            "${ubuntu_core_initramfs}",
            "create-initrd",
            "--root ${UC_INITRD_DEB}",
            "--kernelver=${KERNEL_RELEASE}",
            "--kerneldir ${SNAPCRAFT_PART_INSTALL}/lib/modules/${KERNEL_RELEASE}",
            "--firmwaredir ${SNAPCRAFT_STAGE}/firmware",
            "--skeleton ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs",
            "--output ${SNAPCRAFT_PART_INSTALL}/initrd.img",
        ],
    ),
    "ln $(ls ${SNAPCRAFT_PART_INSTALL}/initrd.img*) ${SNAPCRAFT_PART_INSTALL}/initrd.img",
]

_install_config_cmd = [
    'echo "Installing kernel config..."',
    "ln -f ${SNAPCRAFT_PART_BUILD}/.config ${SNAPCRAFT_PART_INSTALL}/config-${KERNEL_RELEASE}",
]

_finalize_install_cmd = [
    'echo "Finalizing install directory..."',
    "mv ${SNAPCRAFT_PART_INSTALL}/lib/modules ${SNAPCRAFT_PART_INSTALL}/",
    "rm ${SNAPCRAFT_PART_INSTALL}/modules/*/build ${SNAPCRAFT_PART_INSTALL}/modules/*/source",
    " ".join(
        [
            "[ -d ${SNAPCRAFT_PART_INSTALL}/lib/firmware ]",
            "&&",
            "mv ${SNAPCRAFT_PART_INSTALL}/lib/firmware ${SNAPCRAFT_PART_INSTALL}",
        ],
    ),
    "ln -sf ../modules ${SNAPCRAFT_PART_INSTALL}/lib/modules",
    "ln -sf ../firmware ${SNAPCRAFT_PART_INSTALL}/lib/firmware",
]

_clone_zfs_cmd = [
    "if [ ! -d ${SNAPCRAFT_PART_BUILD}/zfs ]; then",
    '\techo "cloning zfs..."',
    "\tgit clone --depth=1 https://github.com/openzfs/zfs ${SNAPCRAFT_PART_BUILD}/zfs -b master",
    "fi",
]

_build_zfs_cmd = [
    'echo "Building zfs modules..."',
    "cd ${SNAPCRAFT_PART_BUILD}/zfs",
    "./autogen.sh",
    " ".join(
        [
            "./configure",
            "--with-linux=${KERNEL_SRC}",
            "--with-linux-obj=${SNAPCRAFT_PART_BUILD}",
            "--with-config=kernel",
            "--host=${SNAPCRAFT_ARCH_TRIPLET}",
        ],
    ),
    "make -j$(nproc)",
    "make install DESTDIR=${SNAPCRAFT_PART_INSTALL}/zfs",
    'release_version="$(ls ${SNAPCRAFT_PART_INSTALL}/modules)"',
    " ".join(
        [
            "mv",
            "${SNAPCRAFT_PART_INSTALL}/zfs/lib/modules/${release_version}/extra",
            "${SNAPCRAFT_PART_INSTALL}/modules/${release_version}",
        ],
    ),
    "rm -rf ${SNAPCRAFT_PART_INSTALL}/zfs",
    'echo "Rebuilding module dependencies"',
    "depmod -b ${SNAPCRAFT_PART_INSTALL} ${release_version}",
]

_build_perf_cmd = [
    'echo "Building perf binary..."',
    'mkdir -p "${SNAPCRAFT_PART_BUILD}/tools/perf"',
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${SNAPCRAFT_PART_BUILD}",
            '-C "${SNAPCRAFT_PART_SRC}/tools/perf"',
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
            "CROSS_COMPILE=${SNAPCRAFT_ARCH_TRIPLET}-",
            '-C "${SNAPCRAFT_PART_SRC}/tools/perf"',
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
