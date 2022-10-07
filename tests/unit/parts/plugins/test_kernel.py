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
import platform
import sys

import pytest
from craft_parts import Part, PartInfo, ProjectInfo

from snapcraft.parts.plugins.kernel import KernelPlugin


@pytest.fixture
def setup_method_fixture():
    def _setup_method_fixture(new_dir, properties=None, arch=None):
        if properties is None:
            properties = {}
        # Ensure that the PPA is not added so we don't cause side-effects
        properties["kernel-add-ppa"] = False

        part = Part("kernel", {})

        if arch is None:
            project_info = ProjectInfo(application_name="test", cache_dir=new_dir)
        else:
            project_info = ProjectInfo(
                application_name="test", cache_dir=new_dir, arch="armv7l"
            )

        part_info = PartInfo(project_info=project_info, part=part)

        return KernelPlugin(
            properties=KernelPlugin.properties_class.unmarshal(properties),
            part_info=part_info,
        )

    yield _setup_method_fixture


class TestPluginKernel:
    """
    Kernel plugin tests.
    """

    def test_get_base_build_packages(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        # default initrd compression is zstd
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "gcc",
            "cmake",
            "cryptsetup",
            "dracut-core",
            "kmod",
            "kpartx",
            "zstd",
            "systemd",
        }

    def test_get_base_build_packages_lz4(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-initrd-compression": "lz4",
            },
        )
        assert plugin.get_build_packages() == {
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
        }

    def test_get_base_build_packages_xz(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-initrd-compression": "xz",
            },
        )
        assert plugin.get_build_packages() == {
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
        }

    def test_get_base_build_packages_zfs(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-enable-zfs-support": True,
            },
        )
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "gcc",
            "cmake",
            "cryptsetup",
            "dracut-core",
            "kmod",
            "kpartx",
            "zstd",
            "systemd",
            "autoconf",
            "automake",
            "libblkid-dev",
            "libtool",
            "python3",
        }

    def test_get_base_build_packages_zfs_cross(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-enable-zfs-support": True,
            },
            arch="armv7l",
        )
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "gcc",
            "cmake",
            "cryptsetup",
            "dracut-core",
            "kmod",
            "kpartx",
            "zstd",
            "systemd",
            "autoconf",
            "automake",
            "libblkid-dev",
            "libtool",
            "python3",
            "libc6-dev:armhf",
        }

    def test_get_base_build_packages_efi(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-build-efi-image": "true",
            },
        )
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "gcc",
            "cmake",
            "cryptsetup",
            "dracut-core",
            "kmod",
            "kpartx",
            "zstd",
            "systemd",
            "llvm",
        }

    def test_get_build_snaps(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        assert plugin.get_build_snaps() == set()

    def test_check_configuration_simple(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-image-target": "bzImage",
            },
        )
        opt = plugin.options

        assert opt.kernel_kdefconfig == ["defconfig"]
        assert opt.kernel_kconfigfile is None
        assert opt.kernel_kconfigflavour is None
        assert opt.kernel_kconfigs is None
        assert opt.kernel_image_target == "bzImage"
        assert opt.kernel_with_firmware
        assert opt.kernel_device_trees is None
        assert not opt.kernel_build_efi_image
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert opt.kernel_initrd_modules is None
        assert opt.kernel_initrd_configured_modules is None
        assert opt.kernel_initrd_firmware is None
        assert opt.kernel_initrd_compression is None
        assert opt.kernel_initrd_compression_options is None
        assert opt.kernel_initrd_overlay is None
        assert opt.kernel_initrd_addons is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf
        assert not opt.kernel_add_ppa

    def test_check_configuration_kde_config(self, setup_method_fixture, new_dir):

        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kdefconfig": ["snappy_defconfig"],
                "kernel-image-target": "Image",
                "kernel-with-firmware": True,
            },
        )
        opt = plugin.options

        assert opt.kernel_kdefconfig == ["snappy_defconfig"]
        assert opt.kernel_kconfigfile is None
        assert opt.kernel_kconfigflavour is None
        assert opt.kernel_kconfigs is None
        assert opt.kernel_image_target == "Image"
        assert opt.kernel_with_firmware
        assert opt.kernel_device_trees is None
        assert not opt.kernel_build_efi_image
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert opt.kernel_initrd_modules is None
        assert opt.kernel_initrd_configured_modules is None
        assert opt.kernel_initrd_firmware is None
        assert opt.kernel_initrd_compression is None
        assert opt.kernel_initrd_compression_options is None
        assert opt.kernel_initrd_overlay is None
        assert opt.kernel_initrd_addons is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf
        assert not opt.kernel_add_ppa

    def test_check_configuration_image_target(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kdefconfig": ["snappy_defconfig"],
                "kernel-image-target": {"arm64": "Image", "armhf": "Image.gz"},
                "kernel-with-firmware": True,
                "kernel-initrd-compression": "zstd",
            },
        )
        opt = plugin.options

        assert opt.kernel_kdefconfig == ["snappy_defconfig"]
        assert opt.kernel_kconfigfile is None
        assert opt.kernel_kconfigflavour is None
        assert opt.kernel_kconfigs is None
        assert opt.kernel_image_target == {"arm64": "Image", "armhf": "Image.gz"}
        assert opt.kernel_with_firmware
        assert opt.kernel_device_trees is None
        assert not opt.kernel_build_efi_image
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert opt.kernel_initrd_modules is None
        assert opt.kernel_initrd_configured_modules is None
        assert opt.kernel_initrd_firmware is None
        assert opt.kernel_initrd_compression == "zstd"
        assert opt.kernel_initrd_compression_options is None
        assert opt.kernel_initrd_overlay is None
        assert opt.kernel_initrd_addons is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf
        assert not opt.kernel_add_ppa

    def test_check_configuration_konfig_file(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kconfigfile": "arch/arm64/configs/snappy_defconfig",
                "kernel-with-firmware": True,
                "kernel-kconfigs": ["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
                "kernel-initrd-compression": "lz4",
                "kernel-initrd-compression-options": ["-9", "-l"],
            },
        )
        opt = plugin.options

        assert opt.kernel_kdefconfig == ["defconfig"]
        assert opt.kernel_kconfigfile == "arch/arm64/configs/snappy_defconfig"
        assert opt.kernel_kconfigflavour is None
        assert opt.kernel_kconfigs == ["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"]
        assert opt.kernel_image_target is None
        assert opt.kernel_with_firmware
        assert opt.kernel_device_trees is None
        assert not opt.kernel_build_efi_image
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert opt.kernel_initrd_modules is None
        assert opt.kernel_initrd_configured_modules is None
        assert opt.kernel_initrd_firmware is None
        assert opt.kernel_initrd_compression == "lz4"
        assert opt.kernel_initrd_compression_options == ["-9", "-l"]
        assert opt.kernel_initrd_overlay is None
        assert opt.kernel_initrd_addons is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf
        assert not opt.kernel_add_ppa

    def test_check_out_of_source_build(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        assert plugin.get_out_of_source_build() is True

    def test_check_get_build_environment(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        plugin.kernel_arch = "amd64"

        assert plugin.get_build_environment() == {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET}-",
            "ARCH": plugin.kernel_arch,
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
            "UC_INITRD_DEB": "${CRAFT_PART_BUILD}/ubuntu-core-initramfs",
            "SNAPD_UNPACKED_SNAP": "${CRAFT_PART_BUILD}/unpacked_snapd",
            "KERNEL_BUILD_ARCH_DIR": f"${{CRAFT_PART_BUILD}}/arch/{plugin.kernel_arch}/boot",
            "KERNEL_IMAGE_TARGET": plugin.kernel_image_target,
        }

    def test_check_get_build_environment_compiler_paths_cross(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-compiler-paths": ["gcc-11/bin"],
                "kernel-image-target": {"arm64": "Image", "armhf": "Image.gz"},
            },
            arch="armv7l",
        )

        assert plugin.get_build_environment() == {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET}-",
            "ARCH": "arm",
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
            "UC_INITRD_DEB": "${CRAFT_PART_BUILD}/ubuntu-core-initramfs",
            "SNAPD_UNPACKED_SNAP": "${CRAFT_PART_BUILD}/unpacked_snapd",
            "KERNEL_BUILD_ARCH_DIR": "${CRAFT_PART_BUILD}/arch/arm/boot",
            "KERNEL_IMAGE_TARGET": "Image.gz",
            "PATH": "${CRAFT_STAGE}/gcc-11/bin:${PATH}",
        }

    def test_check_get_build_environment_compiler_paths(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-compiler-paths": ["gcc-11/bin", "gcc-11/sbin"],
            },
        )
        plugin.kernel_arch = "amd64"

        assert plugin.get_build_environment() == {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET}-",
            "ARCH": plugin.kernel_arch,
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
            "UC_INITRD_DEB": "${CRAFT_PART_BUILD}/ubuntu-core-initramfs",
            "SNAPD_UNPACKED_SNAP": "${CRAFT_PART_BUILD}/unpacked_snapd",
            "KERNEL_BUILD_ARCH_DIR": f"${{CRAFT_PART_BUILD}}/arch/{plugin.kernel_arch}/boot",
            "KERNEL_IMAGE_TARGET": plugin.kernel_image_target,
            "PATH": "${CRAFT_STAGE}/gcc-11/bin:${CRAFT_STAGE}/gcc-11/sbin:${PATH}",
        }

    def test_check_get_build_command(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

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
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert not _is_sub_array(build_commands, _build_zfs_cmd)
        assert not _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_defconfig_configs_no_firmware_lz4(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kconfigfile": "arch/arm64/configs/snappy_defconfig",
                "kernel-compiler": "clang",
                "kernel-compiler-parameters": ["-arch", "arm64"],
                "kernel-image-target": "Image",
                "kernel-with-firmware": False,
                "kernel-kconfigs": ["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
                "kernel-initrd-compression": "lz4",
                "kernel-initrd-compression-options": ["-9", "-l"],
                "kernel-initrd-modules": ["dm-crypt", "slimbus"],
                "kernel-initrd-firmware": ["firmware/for/wifi", "firmware/for/webcam"],
                "kernel-initrd-addons": [
                    "usr/bin/cryptsetup",
                    "usr/lib/my-arch/libcrypto.so",
                ],
            },
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
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert not _is_sub_array(build_commands, _build_zfs_cmd)
        assert not _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_config_flavour_configs(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kconfigflavour": "raspi",
                "kernel-kconfigs": ["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
                "kernel-device-trees": ["pi3", "pi3b", "pi4", "cmd4"],
                "kernel-enable-zfs-support": True,
                "kernel-enable-perf": True,
                "kernel-initrd-modules": ["dm-crypt", "slimbus"],
                "kernel-initrd-configured-modules": ["libarc4"],
                "kernel-initrd-overlay": "my-overlay",
                "kernel-build-efi-image": "true",
            },
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
        assert _is_sub_array(build_commands, _install_kernel_dtbs_cmd)
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
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        print(_create_efi_image_cmd)
        print(build_commands)
        assert _is_sub_array(build_commands, _create_efi_image_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert _is_sub_array(build_commands, _build_zfs_cmd)
        assert _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_cross(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kconfigflavour": "raspi",
                "kernel-kconfigs": ["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
                "kernel-image-target": {"arm64": "Image", "armhf": "Image.gz"},
                "kernel-enable-zfs-support": True,
                "kernel-enable-perf": True,
                "kernel-initrd-modules": ["dm-crypt", "slimbus"],
                "kernel-initrd-configured-modules": ["libarc4"],
                "kernel-initrd-overlay": "my-overlay",
            },
            arch="armv7",
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
        assert _is_sub_array(build_commands, _initrd_tool_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workroud_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert _is_sub_array(build_commands, _build_zfs_cmd)
        assert _is_sub_array(build_commands, _build_perf_armhf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)


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


_DEB_ARCH_TRANSLATIONS = {"aarch64": "arm64", "x86_64": "amd64", "armv7l": "armhf"}

_determine_kernel_src = [
    " ".join(
        [
            "[ -d ${CRAFT_PART_SRC}/kernel ]",
            "&&",
            "KERNEL_SRC=${CRAFT_PART_SRC}",
            "||",
            "KERNEL_SRC=${CRAFT_PROJECT_DIR}",
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
    "\t\t\tlink_files ${1} ${f} ${3}",
    "\t\tdone",
    "\t\treturn 0",
    "\tfi",
    "\tif [ -d ${1}/${2} ]; then",
    "\t\tfor f in $(ls ${1}/${2})",
    "\t\tdo",
    "\t\t\tlink_files ${1} ${2}/${f} ${3}",
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
    " ".join(
        [
            "\tdownload_snap_bootstrap",
            f"{_DEB_ARCH_TRANSLATIONS[platform.machine()]}",
            "${SNAPD_UNPACKED_SNAP}",
        ],
    ),
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
    "[ -e ${CRAFT_PART_INSTALL}/modules ] && rm -rf ${CRAFT_PART_INSTALL}/modules",
    "[ -L ${CRAFT_PART_INSTALL}/lib/modules ] && rm -rf ${CRAFT_PART_INSTALL}/lib/modules",
]

_prepare_config_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${CRAFT_PART_BUILD}/.config ]; then",
    "\t make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} defconfig",
    "fi",
]

_prepare_config_defconfig_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${CRAFT_PART_BUILD}/.config ]; then",
    "\t cp arch/arm64/configs/snappy_defconfig ${CRAFT_PART_BUILD}/.config",
    "fi",
]

_prepare_config_flavour_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${CRAFT_PART_BUILD}/.config ]; then",
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
            "> ${CRAFT_PART_BUILD}/.config 2>/dev/null",
        ],
    ),
    "fi",
]

_prepare_config_extra_config_cmd = [
    'echo "Applying extra config...."',
    "echo 'CONFIG_DEBUG_INFO=n\nCONFIG_DM_CRYPT=y' > ${CRAFT_PART_BUILD}/.config_snap",
    "cat ${CRAFT_PART_BUILD}/.config >> ${CRAFT_PART_BUILD}/.config_snap",
    "echo 'CONFIG_DEBUG_INFO=n\nCONFIG_DM_CRYPT=y' >> ${CRAFT_PART_BUILD}/.config_snap",
    "mv ${CRAFT_PART_BUILD}/.config_snap ${CRAFT_PART_BUILD}/.config",
]
_remake_old_config_cmd = [
    'echo "Remaking oldconfig...."',
    "bash -c ' yes \"\" || true' | make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} oldconfig",
]

_remake_old_config_clang_cmd = [
    'echo "Remaking oldconfig...."',
    " ".join(
        [
            "bash -c ' yes \"\" || true' |",
            "make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD}",
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
            "make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD}",
            "ARCH=arm CROSS_COMPILE=${CRAFT_ARCH_TRIPLET}-",
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
        "${CRAFT_PART_BUILD}/.config",
        "${initrd_installed_kernel_modules}",
        "${initrd_configured_kernel_modules}",
        "",
    ],
)

_build_kernel_cmd = [
    "make -j$(nproc) -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} Image.gz modules dtbs",
]

_build_kernel_clang_image_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            'CC="clang"',
            "-arch arm64",
            "Image modules dtbs",
        ],
    ),
]

_build_kernel_armhf_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "ARCH=arm",
            "CROSS_COMPILE=${CRAFT_ARCH_TRIPLET}-",
            "Image.gz modules dtbs",
        ],
    ),
]

_build_kernel_dtbs_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "Image.gz modules pi3.dtb pi3b.dtb pi4.dtb cmd4.dtb",
        ],
    ),
]

_install_kernel_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${CRAFT_PART_INSTALL}/dtbs",
            "firmware_install INSTALL_FW_PATH=${CRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_install_kernel_dtbs_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
            "firmware_install INSTALL_FW_PATH=${CRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_install_kernel_no_firmware_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${CRAFT_PART_INSTALL}/dtbs",
        ],
    ),
]

_install_kernel_no_firmware_clang_cmd = [
    " ".join(
        [
            "make",
            "-j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            'CC="clang"',
            "-arch arm64",
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${CRAFT_PART_INSTALL}/dtbs",
        ],
    ),
]

_install_kernel_armhf_cmd = [
    " ".join(
        [
            "make",
            "-j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "ARCH=arm",
            "CROSS_COMPILE=${CRAFT_ARCH_TRIPLET}-",
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${CRAFT_PART_INSTALL}/dtbs",
            "firmware_install INSTALL_FW_PATH=${CRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_parse_kernel_release_cmd = [
    "KERNEL_RELEASE=$(cat ${CRAFT_PART_BUILD}/include/config/kernel.release)",
]

_intall_kernel_cmd = [
    "[ -e ${CRAFT_PART_INSTALL}/kernel.img ] && rm -rf ${CRAFT_PART_INSTALL}/kernel.img",
    " ".join(
        [
            "ln -f",
            "${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET}",
            "${CRAFT_PART_INSTALL}/${KERNEL_IMAGE_TARGET}-${KERNEL_RELEASE}",
        ],
    ),
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/${KERNEL_IMAGE_TARGET} ${CRAFT_PART_INSTALL}/kernel.img",
    "",
    'echo "Copying System map..."',
    "[ -e ${CRAFT_PART_INSTALL}/System.map ] && rm -rf ${CRAFT_PART_INSTALL}/System.map*",
    "ln -f ${CRAFT_PART_BUILD}/System.map ${CRAFT_PART_INSTALL}/System.map-${KERNEL_RELEASE}",
]

_install_dtbs_cmd = [
    'echo "Copying custom dtbs..."',
    "mkdir -p ${CRAFT_PART_INSTALL}/dtbs",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi3.dtb ${CRAFT_PART_INSTALL}/dtbs/pi3.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi3b.dtb ${CRAFT_PART_INSTALL}/dtbs/pi3b.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi4.dtb ${CRAFT_PART_INSTALL}/dtbs/pi4.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/cmd4.dtb ${CRAFT_PART_INSTALL}/dtbs/cmd4.dtb",
]

_install_initrd_modules_cmd = [
    'echo "Installing ko modules to initrd..."',
    'install_modules=""',
    'echo "Gathering module dependencies..."',
    'install_modules=""',
    "uc_initrd_feature_kernel_modules="
    "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/kernel-modules",
    "mkdir -p ${uc_initrd_feature_kernel_modules}",
    "initramfs_ko_modules_conf=${uc_initrd_feature_kernel_modules}/extra-modules.conf",
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
    "\tif ! link_files ${CRAFT_PART_INSTALL} ${f} ${uc_initrd_feature_firmware}/lib ; then",
    "\t\tif ! link_files ${CRAFT_STAGE} ${f} ${uc_initrd_feature_firmware}/lib ; then",
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
    "\tlink_files ${CRAFT_STAGE} ${a} ${uc_initrd_feature_overlay}",
    "done",
]

_intatll_initrd_overlay_cmd = [
    "link_files ${CRAFT_STAGE} my-overlay ${uc_initrd_feature_overlay}"
]

_prepare_ininird_features_cmd = [
    'echo "Preparing snap-boostrap initrd feature..."',
    "uc_initrd_feature_snap_bootstratp="
    "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snap-bootstrap",
    "mkdir -p ${uc_initrd_feature_snap_bootstratp}",
    " ".join(
        [
            "link_files",
            "${SNAPD_UNPACKED_SNAP} usr/lib/snapd/snap-bootstrap",
            "${uc_initrd_feature_snap_bootstratp}",
        ],
    ),
    "link_files ${SNAPD_UNPACKED_SNAP} usr/lib/snapd/info ${uc_initrd_feature_snap_bootstratp}",
    "cp ${SNAPD_UNPACKED_SNAP}/usr/lib/snapd/info ${CRAFT_PART_INSTALL}/snapd-info",
]

_clean_old_initrd_cmd = [
    "if compgen -G  ${CRAFT_PART_INSTALL}/initrd.img* >  /dev/null; then",
    "\trm -rf ${CRAFT_PART_INSTALL}/initrd.img*",
    "fi",
]

_initrd_tool_cmd = [
    "ubuntu_core_initramfs=${UC_INITRD_DEB}/usr/bin/ubuntu-core-initramfs",
]
_update_initrd_compression_cmd = [
    'echo "Updating compression command to be used for initrd"',
    "sed -i 's/zstd -1 -T0/lz4 -9 -l/g' ${ubuntu_core_initramfs}",
]

_initrd_tool_workroud_cmd = [
    "for feature in kernel-modules snap-bootstrap uc-firmware uc-overlay",
    "do",
    " ".join(
        [
            "\tlink_files ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/${feature}",
            '"*"',
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/main",
        ],
    ),
    "done",
    " ".join(
        [
            "cp",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/kernel-modules/extra-modules.conf",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/modules/main/extra-modules.conf",
        ],
    ),
]

_create_inird_cmd = [
    " ".join(
        [
            "${ubuntu_core_initramfs}",
            "create-initrd",
            "--kernelver=${KERNEL_RELEASE}",
            "--kerneldir ${CRAFT_PART_INSTALL}/lib/modules/${KERNEL_RELEASE}",
            "--firmwaredir ${CRAFT_STAGE}/firmware",
            "--skeleton ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs",
            "--output ${CRAFT_PART_INSTALL}/initrd.img",
        ],
    ),
    "ln $(ls ${CRAFT_PART_INSTALL}/initrd.img*) ${CRAFT_PART_INSTALL}/initrd.img",
]

_create_efi_image_cmd = [
    " ".join(
        [
            "stub_p=$(find ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/efi/",
            "-maxdepth 1 -name",
            "'linux*.efi.stub'",
            "-printf '%f\\n')",
        ],
    ),
    " ".join(
        [
            "${ubuntu_core_initramfs}",
            "create-efi",
            "--kernelver=${KERNEL_RELEASE}",
            "--stub",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/efi/${stub_p}",
            "--sbat",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/efi/sbat.txt",
            "--key",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key"
            "--cert",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem"
            "--initrd",
            "${CRAFT_PART_INSTALL}/initrd.img",
            "--kernel",
            "${CRAFT_PART_INSTALL}/${KERNEL_IMAGE_TARGET}-${KERNEL_RELEASE}",
            "--output",
            "${CRAFT_PART_INSTALL}/kernel.efi",
        ],
    ),
    "ln $(ls ${CRAFT_PART_INSTALL}/kernel.efi*) ${CRAFT_PART_INSTALL}/kernel.efi",
]

_install_config_cmd = [
    'echo "Installing kernel config..."',
    "ln -f ${CRAFT_PART_BUILD}/.config ${CRAFT_PART_INSTALL}/config-${KERNEL_RELEASE}",
]

_finalize_install_cmd = [
    'echo "Finalizing install directory..."',
    "mv ${CRAFT_PART_INSTALL}/lib/modules ${CRAFT_PART_INSTALL}/",
    "rm ${CRAFT_PART_INSTALL}/modules/*/build ${CRAFT_PART_INSTALL}/modules/*/source",
    " ".join(
        [
            "[ -d ${CRAFT_PART_INSTALL}/lib/firmware ]",
            "&&",
            "mv ${CRAFT_PART_INSTALL}/lib/firmware ${CRAFT_PART_INSTALL}",
        ],
    ),
    "ln -sf ../modules ${CRAFT_PART_INSTALL}/lib/modules",
    "ln -sf ../firmware ${CRAFT_PART_INSTALL}/lib/firmware",
]

_clone_zfs_cmd = [
    "if [ ! -d ${CRAFT_PART_BUILD}/zfs ]; then",
    '\techo "cloning zfs..."',
    "\tgit clone --depth=1 https://github.com/openzfs/zfs ${CRAFT_PART_BUILD}/zfs -b master",
    "fi",
]

_build_zfs_cmd = [
    'echo "Building zfs modules..."',
    "cd ${CRAFT_PART_BUILD}/zfs",
    "./autogen.sh",
    " ".join(
        [
            "./configure",
            "--with-linux=${KERNEL_SRC}",
            "--with-linux-obj=${CRAFT_PART_BUILD}",
            "--with-config=kernel",
            "--host=${CRAFT_ARCH_TRIPLET}",
        ],
    ),
    "make -j$(nproc)",
    "make install DESTDIR=${CRAFT_PART_INSTALL}/zfs",
    'release_version="$(ls ${CRAFT_PART_INSTALL}/modules)"',
    " ".join(
        [
            "mv",
            "${CRAFT_PART_INSTALL}/zfs/lib/modules/${release_version}/extra",
            "${CRAFT_PART_INSTALL}/modules/${release_version}",
        ],
    ),
    "rm -rf ${CRAFT_PART_INSTALL}/zfs",
    'echo "Rebuilding module dependencies"',
    "depmod -b ${CRAFT_PART_INSTALL} ${release_version}",
]

_build_perf_cmd = [
    'echo "Building perf binary..."',
    'mkdir -p "${CRAFT_PART_BUILD}/tools/perf"',
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            '-C "${CRAFT_PART_SRC}/tools/perf"',
            'O="${CRAFT_PART_BUILD}/tools/perf"',
        ],
    ),
    'install -Dm0755 "${CRAFT_PART_BUILD}/tools/perf/perf" "${CRAFT_PART_INSTALL}/bin/perf"',
]

_build_perf_armhf_cmd = [
    'echo "Building perf binary..."',
    'mkdir -p "${CRAFT_PART_BUILD}/tools/perf"',
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "ARCH=arm",
            "CROSS_COMPILE=${CRAFT_ARCH_TRIPLET}-",
            '-C "${CRAFT_PART_SRC}/tools/perf"',
            'O="${CRAFT_PART_BUILD}/tools/perf"',
        ],
    ),
    'install -Dm0755 "${CRAFT_PART_BUILD}/tools/perf/perf" "${CRAFT_PART_INSTALL}/bin/perf"',
]
