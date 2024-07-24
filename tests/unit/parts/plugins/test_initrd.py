# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

import os
import subprocess
import textwrap

import pytest
from craft_parts import Part, PartInfo, ProjectInfo
from pydantic import ValidationError

from snapcraft.parts.plugins import InitrdPlugin


@pytest.fixture
def setup_method_fixture():
    def _setup_method_fixture(new_dir, properties=None, arch="", base="core24"):
        if properties is None:
            properties = {}

        part = Part("initrd", {})

        project_info = ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base=base,
            confinement="strict",
            project_base=base,
            cache_dir=new_dir,
            arch=arch,
        )

        part_info = PartInfo(project_info=project_info, part=part)

        return InitrdPlugin(
            properties=InitrdPlugin.properties_class.unmarshal(properties),
            part_info=part_info,
        )

    yield _setup_method_fixture


class TestPluginInitrd:
    """
    Initrd plugin tests.
    """

    def test_get_base_build_packages(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        # default initrd compression is zstd
        assert plugin.get_build_packages() == {
            "curl",
            "dracut-core",
            "fakechroot",
            "fakeroot",
        }

    def test_get_base_build_packages_armhf(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            arch="armhf",
        )
        if os.getuid() == 0:
            assert plugin.get_build_packages() == {
                "curl",
                "dracut-core",
                "fakechroot",
                "fakeroot",
            }
        else:
            assert plugin.get_build_packages() == {
                "curl",
                "dracut-core",
                "fakechroot",
                "fakeroot",
                "libfakechroot:armhf",
                "libfakeroot:armhf",
            }

    def test_get_build_snaps(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        assert plugin.get_build_snaps() == set()

    def test_check_configuration_simple(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={},
        )
        opt = plugin.options

        assert not opt.initrd_build_efi_image
        assert opt.initrd_efi_image_key is None
        assert opt.initrd_efi_image_cert is None
        assert opt.initrd_modules is None
        assert opt.initrd_configured_modules is None
        assert opt.initrd_firmware is None
        assert opt.initrd_compression is None
        assert opt.initrd_compression_options is None
        assert opt.initrd_overlay is None
        assert opt.initrd_addons is None

    def test_check_configuration_zstd_compression(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-compression": "zstd",
            },
        )
        opt = plugin.options

        assert not opt.initrd_build_efi_image
        assert opt.initrd_efi_image_key is None
        assert opt.initrd_efi_image_cert is None
        assert opt.initrd_modules is None
        assert opt.initrd_configured_modules is None
        assert opt.initrd_firmware is None
        assert opt.initrd_compression == "zstd"
        assert opt.initrd_compression_options is None
        assert opt.initrd_overlay is None
        assert opt.initrd_addons is None

    def test_check_configuration_lz4_custom_compression(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-compression": "lz4",
                "initrd-compression-options": ["-9", "-l"],
            },
        )
        opt = plugin.options

        assert not opt.initrd_build_efi_image
        assert opt.initrd_efi_image_key is None
        assert opt.initrd_efi_image_cert is None
        assert opt.initrd_modules is None
        assert opt.initrd_configured_modules is None
        assert opt.initrd_firmware is None
        assert opt.initrd_compression == "lz4"
        assert opt.initrd_compression_options == ["-9", "-l"]
        assert opt.initrd_overlay is None
        assert opt.initrd_addons is None

    def test_check_configuration_efi_missing_efi_image(
        self, setup_method_fixture, new_dir
    ):
        try:
            setup_method_fixture(
                new_dir,
                properties={
                    "initrd-efi-image-key": "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key",
                    "initrd-efi-image-cert": "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem",
                },
            )
        except ValidationError as err:
            error_list = err.errors()
            assert len(error_list) == 1
            error = error_list[0]
            assert (
                "initrd-efi-image-key and initrd-efi-image-cert must be"
                in error.get("msg")
            )

    def test_check_configuration_efi_missing_key(self, setup_method_fixture, new_dir):
        try:
            setup_method_fixture(
                new_dir,
                properties={
                    "initrd-efi-image-cert": "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem",
                },
            )
        except ValidationError as err:
            error_list = err.errors()
            assert len(error_list) == 1
            error = error_list[0]
            assert (
                "initrd-efi-image-key and initrd-efi-image-cert must be"
                in error.get("msg")
            )

    def test_check_configuration_efi_missing_cert(self, setup_method_fixture, new_dir):
        try:
            setup_method_fixture(
                new_dir,
                properties={
                    "initrd-efi-image-cert": "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem",
                },
            )
        except ValidationError as err:
            error_list = err.errors()
            assert len(error_list) == 1
            error = error_list[0]
            assert (
                "initrd-efi-image-key and initrd-efi-image-cert must be"
                in error.get("msg")
            )

    def test_check_configuration_efi_happy(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-build-efi-image": True,
                "initrd-efi-image-key": "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key",
                "initrd-efi-image-cert": "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem",
            },
        )
        opt = plugin.options

        assert opt.initrd_build_efi_image
        assert (
            opt.initrd_efi_image_key
            == "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key"
        )
        assert (
            opt.initrd_efi_image_cert
            == "/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem"
        )
        assert opt.initrd_modules is None
        assert opt.initrd_configured_modules is None
        assert opt.initrd_firmware is None
        assert opt.initrd_compression is None
        assert opt.initrd_compression_options is None
        assert opt.initrd_overlay is None
        assert opt.initrd_addons is None

    def test_check_out_of_source_build(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        assert plugin.get_out_of_source_build() is True

    def test_check_get_build_environment(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

        assert plugin.get_build_environment() == {
            "UC_INITRD_ROOT_NAME": "uc-initramfs-build-root",
            "UC_INITRD_ROOT": "${CRAFT_PART_SRC}/${UC_INITRD_ROOT_NAME}",
            "KERNEL_MODULES": "${CRAFT_STAGE}/modules",
            "KERNEL_FIRMWARE": "${CRAFT_STAGE}/firmware",
            "UBUNTU_SERIES": "noble",
            "UBUNTU_CORE_BASE": "core24",
        }

    def test_check_get_build_environment_core22(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir, base="core22")

        assert plugin.get_build_environment() == {
            "UC_INITRD_ROOT_NAME": "uc-initramfs-build-root",
            "UC_INITRD_ROOT": "${CRAFT_PART_SRC}/${UC_INITRD_ROOT_NAME}",
            "KERNEL_MODULES": "${CRAFT_STAGE}/modules",
            "KERNEL_FIRMWARE": "${CRAFT_STAGE}/firmware",
            "UBUNTU_SERIES": "jammy",
            "UBUNTU_CORE_BASE": "core22",
        }

    def test_check_get_build_command(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()

        assert _is_sub_array(build_commands, _initrd_modules_empty_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _setup_ubuntu_base_chroot_fnc)
        assert _is_sub_array(build_commands, _chroot_add_snappy_dev_ppa_fnc)
        assert _is_sub_array(build_commands, _chroot_run_cmd_fnc)
        assert _is_sub_array(build_commands, _setup_initrd_chroot_fnc)
        assert _is_sub_array(build_commands, _check_for_stage_firmware_cmd)
        assert _is_sub_array(build_commands, _setup_initrd_build_env_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert not _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_lz4_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_cp_modules_conf_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_inird_cmd)
        assert not _is_sub_array(build_commands, _create_efi_image_cmd)

    def test_check_get_build_command_custom_comp_modules_firmw_addons(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-compression": "lz4",
                "initrd-compression-options": ["-9", "-l"],
                "initrd-modules": ["dm-crypt", "slimbus"],
                "initrd-firmware": ["firmware/for/wifi", "firmware/for/webcam"],
                "initrd-addons": [
                    "usr/bin/cryptsetup",
                    "usr/lib/my-arch/libcrypto.so",
                ],
            },
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _setup_ubuntu_base_chroot_fnc)
        assert _is_sub_array(build_commands, _chroot_add_snappy_dev_ppa_fnc)
        assert _is_sub_array(build_commands, _chroot_run_cmd_fnc)
        assert _is_sub_array(build_commands, _setup_initrd_chroot_fnc)
        assert _is_sub_array(build_commands, _check_for_stage_firmware_cmd)
        assert _is_sub_array(build_commands, _setup_initrd_build_env_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert not _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert _is_sub_array(build_commands, _update_initrd_compression_lz4_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_cp_modules_conf_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_inird_cmd)
        assert not _is_sub_array(build_commands, _create_efi_image_cmd)

    def test_check_get_build_command_comp_modules_modules_configured_overlay(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-modules": ["dm-crypt", "slimbus"],
                "initrd-configured-modules": ["libarc4"],
                "initrd-overlay": "my-overlay",
                "initrd-compression": "gz",
            },
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _setup_ubuntu_base_chroot_fnc)
        assert _is_sub_array(build_commands, _chroot_add_snappy_dev_ppa_fnc)
        assert _is_sub_array(build_commands, _chroot_run_cmd_fnc)
        assert _is_sub_array(build_commands, _setup_initrd_chroot_fnc)
        assert _is_sub_array(build_commands, _check_for_stage_firmware_cmd)
        assert _is_sub_array(build_commands, _setup_initrd_build_env_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert _is_sub_array(build_commands, _update_initrd_compression_gz_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_cp_modules_conf_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_inird_cmd)
        assert not _is_sub_array(build_commands, _create_efi_image_cmd)

    def test_check_get_build_command_modules_modules_configured_overlay_cross(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-modules": ["dm-crypt", "slimbus"],
                "initrd-configured-modules": ["libarc4"],
                "initrd-overlay": "my-overlay",
            },
            arch="armhf",
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _setup_ubuntu_base_chroot_fnc)
        assert _is_sub_array(build_commands, _chroot_add_snappy_dev_ppa_fnc)
        assert _is_sub_array(build_commands, _chroot_run_cmd_fnc)
        assert _is_sub_array(build_commands, _setup_initrd_chroot_fnc)
        assert _is_sub_array(build_commands, _check_for_stage_firmware_cmd)
        assert _is_sub_array(build_commands, _setup_initrd_build_env_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_lz4_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_cp_modules_conf_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_inird_cmd)
        assert not _is_sub_array(build_commands, _create_efi_image_cmd)

    @pytest.mark.parametrize("arch", ["arm64", "armhf", "riscv64", "amd64"])
    def test_check_arch(self, arch, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir, arch=arch)

        assert plugin._target_arch == _DEB_ARCH_TRANSLATIONS[arch]

    def test_check_get_build_command_efi(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-build-efi-image": "true",
            },
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _initrd_modules_empty_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _setup_ubuntu_base_chroot_fnc)
        assert _is_sub_array(build_commands, _chroot_add_snappy_dev_ppa_fnc)
        assert _is_sub_array(build_commands, _chroot_run_cmd_fnc)
        assert _is_sub_array(build_commands, _setup_initrd_chroot_fnc)
        assert _is_sub_array(build_commands, _check_for_stage_firmware_cmd)
        assert _is_sub_array(build_commands, _setup_initrd_build_env_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert not _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_lz4_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_cp_modules_conf_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_inird_cmd)
        assert _is_sub_array(build_commands, _create_efi_image_cmd)

    def test_check_get_build_command_efi_custom_keys(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-build-efi-image": "true",
                "initrd-efi-image-key": "/home/ubuntu/efi-signing-key.key",
                "initrd-efi-image-cert": "/home/ubuntu/efi-signing-key.pem",
            },
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _initrd_modules_empty_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _setup_ubuntu_base_chroot_fnc)
        assert _is_sub_array(build_commands, _chroot_add_snappy_dev_ppa_fnc)
        assert _is_sub_array(build_commands, _chroot_run_cmd_fnc)
        assert _is_sub_array(build_commands, _setup_initrd_chroot_fnc)
        assert _is_sub_array(build_commands, _check_for_stage_firmware_cmd)
        assert _is_sub_array(build_commands, _setup_initrd_build_env_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_initrd_modules_cmd)
        assert _is_sub_array(build_commands, _initrd_overlay_features_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_firmware_cmd)
        assert not _is_sub_array(build_commands, _install_initrd_addons_cmd)
        assert not _is_sub_array(build_commands, _intatll_initrd_overlay_cmd)
        assert _is_sub_array(build_commands, _prepare_ininird_features_cmd)
        assert _is_sub_array(build_commands, _clean_old_initrd_cmd)
        assert not _is_sub_array(build_commands, _update_initrd_compression_lz4_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_workaroud_cmd)
        assert _is_sub_array(build_commands, _initrd_tool_cp_modules_conf_cmd)
        assert _is_sub_array(build_commands, _create_inird_cmd)
        assert _is_sub_array(build_commands, _install_inird_cmd)
        assert _is_sub_array(build_commands, _create_efi_image_custom_keys_cmd)
        assert _is_sub_array(build_commands, _create_efi_prepare_custom_keys_cmd)


def subprocess_callback_function(process):
    process.returncode = 1
    raise subprocess.CalledProcessError(
        1, cmd="apt", output="command failed", stderr="command failed"
    )


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
    "arm64": "arm64",
    "armhf": "armhf",
    "riscv64": "riscv64",
    "amd64": "amd64",
}

_SNAPPY_DEV_KEY_FINGERPRINT = "F1831DDAFC42E99D"

_initrd_modules_empty_cmd = [
    textwrap.dedent(
        """
        # list of kernel modules to be installed in the initrd
        initrd_installed_kernel_modules=""
        """
    )
]

_initrd_modules_cmd = [
    textwrap.dedent(
        """
        # list of kernel modules to be installed in the initrd
        initrd_installed_kernel_modules="dm-crypt slimbus"
        """
    )
]

_initrd_configured_modules_empty_cmd = [
    textwrap.dedent(
        """
        # list of kernel modules in the initrd to be auto loaded
        # any module in this list implies it will be added to initrd
        initrd_configured_kernel_modules=""
        """
    )
]

_initrd_configured_modules_cmd = [
    textwrap.dedent(
        """
        # list of kernel modules in the initrd to be auto loaded
        # any module in this list implies it will be added to initrd
        initrd_configured_kernel_modules="libarc4"
        """
    )
]

_link_files_fnc = [
    textwrap.dedent(
        """
        # link files helper, accept wild cards
        # 1: reference dir, 2: file(s) including wild cards, 3: dst dir
        # 4: quiet mode [ "-quiet" ] (optional)
        link_files() {
            set +x
            link_files_impl "${@}"
            local retVal=$?
            set -x
            return ${retVal}
        }

        # link files helper implementation, accept wild cards
        # 1: reference dir, 2: file(s) including wild cards, 3: dst dir
        # 4: quiet mode [ "-quiet" ] (optional)
        link_files_impl() {
            if [ -z "${2}" ]; then
                return 0
            fi
            local quiet="${4:--noisy}"
            local f
            if [ "${2}" = "*" ]; then
                while IFS= read -r -d $'\\0' f
                do
                    link_files_impl "${1}" "${f}" "${3}" "${quiet}"
                done < <(find "${1}" -maxdepth 1 -mindepth 1 -printf '%P\\0')
                return 0
            fi
            if [ -d "${1}/${2}" ]; then
                while IFS= read -r -d $'\\0' f
                do
                    link_files_impl "${1}" "${2}/${f}" "${3}" "${quiet}"
                done < <(find "${1}/${2}" -maxdepth 1 -mindepth 1 -printf '%P\\0')
                return 0
            fi

            local found searchdir basename rel_path dir_path
            searchdir=$(dirname "${2}")
            basename=$(basename "${2}")
            if ! compgen -G "${1}/${searchdir}" > /dev/null; then
                echo "search pattern <${1}/${searchdir}> <${basename}> does not exist"
                return 1
            fi
            # shellcheck disable=SC2086 # SC2086 does not apply, searchdir can contain wild cards, it cannot be quoted
            while IFS= read -r -d $'\\0' f
            do
                if [[ -d "${f}" ]]; then
                    link_files_impl "${1}" "${rel_path}" "${3}" "${quiet}"
                else
                    if [[ -L "${f}" ]]; then
                        rel_path=$( realpath --no-symlinks --relative-to="${1}" "${f}" )
                    else
                        rel_path=$( realpath -se --relative-to="${1}" "${f}" )
                    fi
                    dir_path=$(dirname "${rel_path}")
                    mkdir -p "${3}/${dir_path}"
                    [ "${quiet}" != "-quiet" ] && echo "installing ${f} to ${3}/${dir_path}"
                    ln -f "${f}" "${3}/${dir_path}"
                fi
                found="yes"
            done < <(find "${1}"/${searchdir} -maxdepth 1 -mindepth 1 -name "${basename}" -printf '%p\\0')
            if [ "yes" = "${found:-}" ]; then
                return 0
            else
                return 1
            fi
        }
        """
    )
]


_setup_ubuntu_base_chroot_fnc = [
    textwrap.dedent(
        """
        # setup chroot from Ubuntu Base
        # 1: work dir, 2: ubuntu series
        setup_chroot_base() {
            local work_dir="${1}"
            local series="${2}"
            local ubuntu_base="${work_dir}/ubuntu-base-${series}-${CRAFT_ARCH_BUILD_FOR}.tar.gz"
            rm -rf "${ubuntu_base}"
            curl \\
                "https://cdimage.ubuntu.com/ubuntu-base/${series}/daily/current/${series}-base-${CRAFT_ARCH_BUILD_FOR}.tar.gz" \\
                --output "${ubuntu_base}"
            rm -rf "${UC_INITRD_ROOT}"
            mkdir -p "${UC_INITRD_ROOT}"
            tar --extract --file "${ubuntu_base}" --directory "${UC_INITRD_ROOT}"
            cp --no-dereference /etc/resolv.conf "${UC_INITRD_ROOT}/etc/resolv.conf"
            # if not running as root, setup build root env with fakechroot, fakeroot
            if [ "$(whoami)" != "root" ]; then
                cp --no-dereference --recursive \\
                    /usr/lib/"${CRAFT_ARCH_TRIPLET_BUILD_FOR}"/fakechroot \\
                    "${UC_INITRD_ROOT}/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
                cp --no-dereference --recursive \\
                    /usr/lib/"${CRAFT_ARCH_TRIPLET_BUILD_FOR}"/libfakeroot \\
                    "${UC_INITRD_ROOT}/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}"
            fi
            # setup /dev/null as it's used to mask systemd service files
            touch "${UC_INITRD_ROOT}/dev/null"
        }
        """
    )
]

_chroot_add_snappy_dev_ppa_fnc = [
    textwrap.dedent(
        """
        # add snappy-dev/image ppa to the chroot
        # import ppa keys for the ppa
        # 1: work dir, 2: ubuntu series, 3: ppa fingerprint
        chroot_add_snappy_dev_ppa() {
            local work_dir="${1}"
            local series="${2}"
            local key_fingerprint="${3}"
            local source_file="snappy-dev-image.sources"
            local key_file="/etc/apt/keyrings/snappy-dev-image.gpg"
            local chroot_home="${UC_INITRD_ROOT}"
            local gnupg_home="${work_dir}/.gnupg"
            local snappy_key="${gnupg_home}/snappy-dev.kbx"
            rm -rf "${gnupg_home}"
            mkdir --mode 700 "${gnupg_home}"
            gpg \\
                --homedir "${gnupg_home}" \\
                --no-default-keyring \\
                --keyring "${snappy_key}" \\
                --keyserver keyserver.ubuntu.com \\
                --recv-keys "${key_fingerprint}"

            mkdir -p "$(dirname "${chroot_home}/${key_file}")"
            rm -rf "${chroot_home:?}/${key_file}"
            gpg \\
                --homedir "${gnupg_home}" \\
                --no-default-keyring \\
                --keyring "${snappy_key}" \\
                --export \\
                --out "${chroot_home}/${key_file}"

            tee "${chroot_home}/etc/apt/sources.list.d/${source_file}" <<EOF
        Types: deb
        URIs: https://ppa.launchpadcontent.net/snappy-dev/image/ubuntu/
        Suites: ${series}
        Components: main
        Signed-By: ${key_file}
        EOF
        }
        """
    )
]


_chroot_run_cmd_fnc = [
    textwrap.dedent(
        """
        # clean any existing mounts for the chroot function
        _clean_chroot() {
            local chroot_home="${1}"
            if [ -z "${chroot_home}" ]; then
                echo "Missing chroot home to clean"
                return
            fi
            for m in dev/pts dev/null dev/zero dev/full dev/random dev/urandom dev/tty dev proc run sys
            do
                if grep "${chroot_home}/${m}" /proc/self/mounts > /dev/null; then
                    umount "${chroot_home}/${m}"
                fi
            done
        }

        _chroot_configured="no"
        # setup necessary mounts for the chroot function
        _setup_chroot() {
            if [ "${_chroot_configured}" = "yes" ]; then
                return
            fi
            local chroot_home="${1}"
            for m in proc run sys dev dev/pts dev/null dev/zero dev/full dev/random dev/urandom dev/tty
            do
                mount --bind "/${m}" "${chroot_home}/${m}"
            done
            _chroot_configured="yes"
        }

        # run command in true chroot
        _run_truechroot() {
            local chroot_home="${1}"
            local cmd="${2}"
            trap "_clean_chroot ${chroot_home}" EXIT
            _setup_chroot "${chroot_home}"
            chroot "${chroot_home}" /bin/bash -c "${cmd}"
        }

        # run command in fake chroot
        _run_fakechroot() {
            local chroot_home="${1}"
            local cmd="${2}"
            if [ "${UBUNTU_SERIES}" = "focal" ] || [ "${UBUNTU_SERIES}" = "jammy" ]; then
                ld_path="${LD_LIBRARY_PATH}:/usr/lib/systemd:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/fakechroot:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libfakeroot"
            else
                ld_path="${LD_LIBRARY_PATH}:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/systemd:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/fakechroot:/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libfakeroot"
            fi
            LD_LIBRARY_PATH="${ld_path}" fakechroot fakeroot chroot "${chroot_home}" /bin/bash -c "${cmd}"
        }

        # run command with chroot
        # 1: chroot home, 2: command to run
        run_chroot() {
            local chroot_home="${1}"
            local cmd="${2}"
            # use true chroot if we have root permissions
            if [ "$(whoami)" = "root" ]; then
                _run_truechroot "${chroot_home}" "${cmd}"
            else
                _run_fakechroot "${chroot_home}" "${cmd}"
            fi
        }
        """
    )
]

_setup_initrd_chroot_fnc = [
    textwrap.dedent(
        """
        # setup chroot to build Ubuntu Core initrd
        # chroot is based on the Ubuntu Base
        # 1: work dir, 2: ppa fingerprint
        setup_initrd_chroot() {
            set +x
            local work_dir="${1}"
            local ppa_fingerprint="${2}"

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.base" ]; then
                setup_chroot_base "${work_dir}" "${UBUNTU_SERIES}"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.base"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.ppa" ]; then
                run_chroot "${UC_INITRD_ROOT}" "apt-get update"
                run_chroot "${UC_INITRD_ROOT}" "apt-get dist-upgrade -y"
                run_chroot "${UC_INITRD_ROOT}" "apt-get install --no-install-recommends -y ca-certificates debconf-utils libfakeroot lz4 xz-utils zstd"
                if [ "${UBUNTU_SERIES}" = "focal" ] || [ "${UBUNTU_SERIES}" = "jammy" ]; then
                    run_chroot "${UC_INITRD_ROOT}" "apt-get install --no-install-recommends -y systemd"
                else
                    run_chroot "${UC_INITRD_ROOT}" "apt-get install --no-install-recommends -y libsystemd-shared"
                fi
                chroot_add_snappy_dev_ppa "${work_dir}" "${UBUNTU_SERIES}" "${ppa_fingerprint}"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.ppa"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.u-c-i" ]; then
                run_chroot "${UC_INITRD_ROOT}" "apt-get update"
                run_chroot "${UC_INITRD_ROOT}" "echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections"
                run_chroot "${UC_INITRD_ROOT}" "DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y snapd ubuntu-core-initramfs"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.u-c-i"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.firmware" ]; then
                rm -rf "${UC_INITRD_ROOT}"/usr/lib/firmware/*
                link_files "${KERNEL_FIRMWARE}" "*" "${UC_INITRD_ROOT}/usr/lib/firmware"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.firmware"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.modules" ]; then
                rm -rf "${UC_INITRD_ROOT}"/usr/lib/modules/*
                link_files "${KERNEL_MODULES}" "*" "${UC_INITRD_ROOT}/usr/lib/modules"
                # remove potentially dangling source link
                rm -rf ${UC_INITRD_ROOT}/usr/lib/modules/*/build
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.modules"
            fi

            if [ ! -e "${work_dir}/.${UC_INITRD_ROOT_NAME}.os-release" ]; then
                cp "/snap/${UBUNTU_CORE_BASE}/current/etc/os-release" "${UC_INITRD_ROOT}/etc/os-release"
                touch "${work_dir}/.${UC_INITRD_ROOT_NAME}.os-release"
            fi
            set -x
        }
        """
    )
]

_check_for_stage_firmware_cmd = [
    textwrap.dedent(
        """
        [ ! -d "${CRAFT_STAGE}/firmware" ] && \\
            echo -e "firmware directory ${CRAFT_STAGE}/firmware does not exist, ensure part building firmware is run before this part." \\
                && exit 1
        """
    )
]

_setup_initrd_build_env_cmd = [
    textwrap.dedent(
        f"""
        echo "Preparing Ubuntu Core Initrd chroot build environment..."
        setup_initrd_chroot "${{CRAFT_PART_SRC}}" "{_SNAPPY_DEV_KEY_FINGERPRINT}"
        """
    )
]

_parse_kernel_release_cmd = [
    'KERNEL_RELEASE=$(ls "${CRAFT_STAGE}/modules")',
]

_install_initrd_modules_cmd = [
    textwrap.dedent(
        """
        echo "Installing ko modules to initrd..."
        # shellcheck disable=SC2034 #SC2034 does not apply as install_modules not be always used
        install_modules=""
        echo "Gathering module dependencies..."
        uc_initrd_feature_kernel_modules=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/kernel-modules
        mkdir -p "${uc_initrd_feature_kernel_modules}"
        initramfs_ko_modules_conf=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/extra-modules.conf
        rm -rf "${initramfs_ko_modules_conf}"
        for m in ${initrd_installed_kernel_modules} ${initrd_configured_kernel_modules}
        do
            echo "${m}" >> "${initramfs_ko_modules_conf}"
        done
        [ -e "${initramfs_ko_modules_conf}" ] && sort -fu "${initramfs_ko_modules_conf}" -o "${initramfs_ko_modules_conf}"

        echo "Configuring ubuntu-core-initramfs.conf with supported modules"
        echo "If module does not exist, do not include it"
        initramfs_conf_dir=${uc_initrd_feature_kernel_modules}/usr/lib/modules-load.d
        mkdir -p "${initramfs_conf_dir}"
        initramfs_conf=${initramfs_conf_dir}/ubuntu-core-initramfs.conf
        echo "# configured modules" > "${initramfs_conf}"
        # shellcheck disable=SC2013 #SC2013 does not apply as array could be env, or file content
        for m in ${initrd_configured_kernel_modules}
        do
            if [ -n "$(modprobe -n -q --show-depends -d "${CRAFT_STAGE}" -S "${KERNEL_RELEASE}" "${m}")" ] ; then
                echo "${m}" >> "${initramfs_conf}"
            fi
        done
        """
    )
]

_initrd_overlay_features_cmd = [
    textwrap.dedent(
        """
        uc_initrd_feature_firmware=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-firmware
        mkdir -p "${uc_initrd_feature_firmware}"
        uc_initrd_feature_overlay=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/uc-overlay
        mkdir -p "${uc_initrd_feature_overlay}"
        """
    )
]

_install_initrd_firmware_cmd = [
    textwrap.dedent(
        """
        echo "Installing initrd overlay firmware..."
        for fw in firmware/for/wifi firmware/for/webcam
        do
            # firmware can be from kernel build or from stage
            # firmware from kernel build takes preference
            if ! link_files "${CRAFT_PART_INSTALL}" "${fw}" "${uc_initrd_feature_firmware}/usr/lib" ; then
                if ! link_files "${CRAFT_STAGE}" "${fw}" "${uc_initrd_feature_firmware}/usr/lib"; then
                    echo "Missing firmware [${fw}], ignoring it"
                fi
            fi
        done
        """
    )
]

_install_initrd_addons_cmd = [
    textwrap.dedent(
        """
        echo "Installing initrd addons..."
        for a in usr/bin/cryptsetup usr/lib/my-arch/libcrypto.so
        do
            echo "Copy overlay: ${a}"
            link_files "${CRAFT_STAGE}" "${a}" "${uc_initrd_feature_overlay}"
        done
        """
    )
]

_intatll_initrd_overlay_cmd = [
    textwrap.dedent(
        """
        link_files "${CRAFT_STAGE}/my-overlay" "*" "${uc_initrd_feature_overlay}"
        """
    )
]

_prepare_ininird_features_cmd = [
    textwrap.dedent(
        """
        # install selected snap bootstrap
        echo "Preparing snap-boostrap initrd feature..."
        uc_initrd_main_lib_snapd=${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main/usr/lib/snapd
        [ -e "${uc_initrd_main_lib_snapd}/snap-bootstrap" ] && ln -f "${UC_INITRD_ROOT}/usr/lib/snapd/snap-bootstrap" \\
                                                                     "${uc_initrd_main_lib_snapd}/snap-bootstrap"
        [ -e "${uc_initrd_main_lib_snapd}/info" ] && ln -f "${UC_INITRD_ROOT}/usr/lib/snapd/info" \\
                                                           "${uc_initrd_main_lib_snapd}/info"

        cp "${UC_INITRD_ROOT}/usr/lib/snapd/info" "${CRAFT_PART_INSTALL}/snapd-info"
        """
    )
]

_clean_old_initrd_cmd = [
    textwrap.dedent(
        """
        if compgen -G "${CRAFT_PART_INSTALL}/initrd.img*" > /dev/null; then
            rm -rf "${CRAFT_PART_INSTALL}"/initrd.img*
        fi
        """
    )
]

_update_initrd_compression_lz4_cmd = [
    textwrap.dedent(
        """
        echo "Updating compression command to be used for initrd"
        sed -i 's/zstd -1 -T0/lz4 -9 -l/g' "${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs"',
        """
    )
]

_update_initrd_compression_gz_cmd = [
    textwrap.dedent(
        """
        echo "Updating compression command to be used for initrd"
        sed -i 's/zstd -1 -T0/gzip -7/g' "${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs"',
        """
    )
]

_initrd_tool_workaroud_cmd = [
    textwrap.dedent(
        """
        echo "Workaround for bug in ubuntu-core-initramfs"
        for feature in kernel-modules uc-firmware uc-overlay
        do
            link_files "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/${feature}" \\
                "*" \\
                "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main"
        done

        # actual ubuntu-core initramfs build is performed in chroot
        # where tmp is not really tmpfs, avoid excessive use of cp
        # cp "-ar"/"-aR" -> cp "-lR"
        sed -i \\
            -e 's/"cp", "-ar", args./"cp", "-lR", args./g' \\
            -e 's/"cp", "-aR", args./"cp", "-lR", args./g' \\
            ${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs
        if [ "$(whoami)" != "root" ]; then
            # ubuntu-core-initramfs unsets LD_PRELOAD before invoking dracut
            # this leads to escaping of fakechroot, disable this
            sed -i \\
                -e 's/\\(.*\\)proc_env\\["LD_PRELOAD"\\]\\(.*\\)/\\1# proc_env\\["LD_PRELOAD"\\]\\2/g' \\
                ${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs
        fi
        """
    )
]

_initrd_tool_cp_modules_conf_cmd = [
    textwrap.dedent(
        """
        if [ -e "${initramfs_ko_modules_conf}" ]; then
            if [ "${UBUNTU_SERIES}" = "focal" ]; then
                cp "${initramfs_ko_modules_conf}" \\
                   "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/main/extra-modules.conf"
            else
                cp "${initramfs_ko_modules_conf}" \\
                   "${UC_INITRD_ROOT}/usr/lib/ubuntu-core-initramfs/modules/main/extra-modules.conf"
            fi
        fi
        """
    )
]

_create_inird_cmd = [
    textwrap.dedent(
        """
        rm -rf ${UC_INITRD_ROOT}/boot/initrd*
        run_chroot "${UC_INITRD_ROOT}" \\
                "ubuntu-core-initramfs create-initrd --kernelver "${KERNEL_RELEASE}" --output /boot/initrd.img"
        """
    )
]

_install_inird_cmd = [
    textwrap.dedent(
        """
        link_files "${UC_INITRD_ROOT}"/boot "initrd.img*" "${CRAFT_PART_INSTALL}"
        ln -f "$(ls "${CRAFT_PART_INSTALL}"/initrd.img*)" "${CRAFT_PART_INSTALL}"/initrd.img
        """
    )
]

_create_efi_image_cmd = [
    textwrap.dedent(
        """
        echo "Building kernel.efi"
        rm -rf ${UC_INITRD_ROOT}/boot/kernel.efi*
        ln -f "${CRAFT_STAGE}"/kernel.img "${UC_INITRD_ROOT}/boot/kernel.img-${KERNEL_RELEASE}"
        run_chroot "${UC_INITRD_ROOT}" \\
                "ubuntu-core-initramfs create-efi \\
                    --kernelver=${KERNEL_RELEASE} \\
                    --key /usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key \\
                    --cert /usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem \\
                    --initrd /boot/initrd.img \\
                    --kernel /boot/kernel.img \\
                    --output /boot/kernel.efi"

        echo "Installing created kernel.efi image"
        link_files "${UC_INITRD_ROOT}/boot" "kernel.efi*" "${CRAFT_PART_INSTALL}"
        ln -f "$(ls "${CRAFT_PART_INSTALL}"/kernel.efi*)" "${CRAFT_PART_INSTALL}/kernel.efi"
        rm "${CRAFT_PART_INSTALL}"/initrd.img*
        """
    )
]

_create_efi_image_custom_keys_cmd = [
    textwrap.dedent(
        """
        echo "Building kernel.efi"
        rm -rf ${UC_INITRD_ROOT}/boot/kernel.efi*
        ln -f "${CRAFT_STAGE}"/kernel.img "${UC_INITRD_ROOT}/boot/kernel.img-${KERNEL_RELEASE}"
        run_chroot "${UC_INITRD_ROOT}" \\
                "ubuntu-core-initramfs create-efi \\
                    --kernelver=${KERNEL_RELEASE} \\
                    --key /root/efi-signing-key.key \\
                    --cert /root/efi-certificate.pem \\
                    --initrd /boot/initrd.img \\
                    --kernel /boot/kernel.img \\
                    --output /boot/kernel.efi"

        echo "Installing created kernel.efi image"
        link_files "${UC_INITRD_ROOT}/boot" "kernel.efi*" "${CRAFT_PART_INSTALL}"
        ln -f "$(ls "${CRAFT_PART_INSTALL}"/kernel.efi*)" "${CRAFT_PART_INSTALL}/kernel.efi"
        rm "${CRAFT_PART_INSTALL}"/initrd.img*
        """
    )
]

_create_efi_prepare_custom_keys_cmd = [
    textwrap.dedent(
        """
        cp --link "/home/ubuntu/efi-signing-key.key" "${UC_INITRD_ROOT}/root/efi-signing-key.key"
        cp --link "/home/ubuntu/efi-signing-key.pem" "${UC_INITRD_ROOT}/root/efi-certificate.pem"
        """
    )
]
