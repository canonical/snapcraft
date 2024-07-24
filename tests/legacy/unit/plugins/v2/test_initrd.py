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

import os
import platform
import textwrap
from dataclasses import dataclass
from typing import List, Optional
from unittest import mock

from testtools import TestCase

from snapcraft_legacy.plugins.v2.initrd import InitrdPlugin


class Initrdv2PluginProperties:
    initrd_modules: Optional[List[str]]
    initrd_configured_modules: Optional[List[str]]
    initrd_firmware: Optional[List[str]]
    initrd_compression: Optional[str]
    initrd_compression_options: Optional[List[str]]
    initrd_overlay: Optional[str]
    initrd_addons: Optional[List[str]]


class TestPluginInitrd(TestCase):
    """
    Legacy initrd plugin tests.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _setup_test(
        self,
        initrdmodules=None,
        initrdconfiguredmodules=None,
        initrdstagefirmware=False,
        initrdfirmware=None,
        initrdcompression=None,
        initrdcompressionoptions=None,
        initrdoverlay=None,
        initrdaddons=None,
        arch=platform.machine(),
    ):
        @dataclass
        class Options(Initrdv2PluginProperties):
            initrd_modules = initrdmodules
            initrd_configured_modules = initrdconfiguredmodules
            initrd_stage_firmware = initrdstagefirmware
            initrd_firmware = initrdfirmware
            initrd_compression = initrdcompression
            initrd_compression_options = initrdcompressionoptions
            initrd_overlay = initrdoverlay
            initrd_addons = initrdaddons

        target_arch = _DEB_ARCH_TRANSLATIONS[arch]
        # plugin used os.getenv("SNAP_ARCH") to determine host arch
        os.environ["SNAP_ARCH"] = _DEB_ARCH_TRANSLATIONS[platform.machine()]

        with mock.patch(
            "snapcraft_legacy.plugins.v2.initrd._get_target_architecture",
            return_value=target_arch,
        ):
            plugin = InitrdPlugin(part_name="initrd", options=Options())

        return plugin

    def test_schema(self):
        self.assertEqual(
            InitrdPlugin.get_schema(),
            {
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
            },
        )

    def test_get_base_build_packages(self):
        plugin = self._setup_test()
        # default initrd compression is zstd
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "curl",
                "fakechroot",
                "fakeroot",
            },
        )

    def test_get_base_build_packages_armhf(self):
        plugin = self._setup_test(arch="armv7l")
        # default initrd compression is zstd
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "curl",
                "fakechroot",
                "fakeroot",
                "libfakechroot:armhf",
                "libfakeroot:armhf",
            },
        )

    def test_get_build_snaps(self):
        plugin = self._setup_test()
        self.assertEqual(plugin.get_build_snaps(), set())

    def test_check_configuration_simple(self):
        plugin = self._setup_test()
        opt = plugin.options

        self.assertEqual(opt.initrd_modules, None)
        self.assertEqual(opt.initrd_configured_modules, None)
        self.assertEqual(opt.initrd_firmware, None)
        self.assertEqual(opt.initrd_compression, None)
        self.assertEqual(opt.initrd_compression_options, None)
        self.assertEqual(opt.initrd_overlay, None)
        self.assertEqual(opt.initrd_addons, None)

    def test_check_configuration_zstd_compression(self):
        plugin = self._setup_test(
            initrdcompression="zstd",
        )
        opt = plugin.options

        self.assertIs(opt.initrd_modules, None)
        self.assertIs(opt.initrd_configured_modules, None)
        self.assertIs(opt.initrd_firmware, None)
        self.assertEqual(opt.initrd_compression, "zstd")
        self.assertIs(opt.initrd_compression_options, None)
        self.assertIs(opt.initrd_overlay, None)
        self.assertIs(opt.initrd_addons, None)

    def test_check_configuration_lz4_custom_compression(self):
        plugin = self._setup_test(
            initrdcompression="lz4",
            initrdcompressionoptions=["-9", "-l"],
        )
        opt = plugin.options

        self.assertIs(opt.initrd_modules, None)
        self.assertIs(opt.initrd_configured_modules, None)
        self.assertIs(opt.initrd_firmware, None)
        self.assertEqual(opt.initrd_compression, "lz4")
        self.assertEqual(opt.initrd_compression_options, ["-9", "-l"])
        self.assertIs(opt.initrd_overlay, None)
        self.assertIs(opt.initrd_addons, None)

    def test_check_get_build_environment(self):
        plugin = self._setup_test()

        self.assertEqual(
            plugin.get_build_environment(),
            {
                "UC_INITRD_ROOT_NAME": "uc-initramfs-build-root",
                "UC_INITRD_ROOT": "${SNAPCRAFT_PART_SRC}/${UC_INITRD_ROOT_NAME}",
                "KERNEL_MODULES": "${SNAPCRAFT_STAGE}/modules",
                "KERNEL_FIRMWARE": "${SNAPCRAFT_STAGE}/firmware",
                "UBUNTU_SERIES": "focal",
                "UBUNTU_CORE_BASE": "core20",
                "CRAFT_ARCH_TRIPLET_BUILD_FOR": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}",
                "CRAFT_ARCH_BUILD_FOR": _DEB_ARCH_TRANSLATIONS[platform.machine()],
                "CRAFT_ARCH_BUILD_ON": _DEB_ARCH_TRANSLATIONS[platform.machine()],
                "CRAFT_STAGE": "${SNAPCRAFT_STAGE}",
                "CRAFT_PART_SRC": "${SNAPCRAFT_PART_SRC}",
                "CRAFT_PART_BUILD": "${SNAPCRAFT_PART_BUILD}",
                "CRAFT_PART_INSTALL": "${SNAPCRAFT_PART_INSTALL}",
            },
        )

    def test_check_get_build_environment_armhf(self):
        plugin = self._setup_test(arch="armv7l")

        self.assertEqual(
            plugin.get_build_environment(),
            {
                "UC_INITRD_ROOT_NAME": "uc-initramfs-build-root",
                "UC_INITRD_ROOT": "${SNAPCRAFT_PART_SRC}/${UC_INITRD_ROOT_NAME}",
                "KERNEL_MODULES": "${SNAPCRAFT_STAGE}/modules",
                "KERNEL_FIRMWARE": "${SNAPCRAFT_STAGE}/firmware",
                "UBUNTU_SERIES": "focal",
                "UBUNTU_CORE_BASE": "core20",
                "CRAFT_ARCH_TRIPLET_BUILD_FOR": "${SNAPCRAFT_ARCH_TRIPLET_BUILD_FOR}",
                "CRAFT_ARCH_BUILD_FOR": "armhf",
                "CRAFT_ARCH_BUILD_ON": _DEB_ARCH_TRANSLATIONS[platform.machine()],
                "CRAFT_STAGE": "${SNAPCRAFT_STAGE}",
                "CRAFT_PART_SRC": "${SNAPCRAFT_PART_SRC}",
                "CRAFT_PART_BUILD": "${SNAPCRAFT_PART_BUILD}",
                "CRAFT_PART_INSTALL": "${SNAPCRAFT_PART_INSTALL}",
            },
        )

    def test_check_get_build_command(self):
        plugin = self._setup_test()

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

    def test_check_get_build_command_custom_comp_modules_firmw_addons(self):
        plugin = self._setup_test(
            initrdcompression="lz4",
            initrdcompressionoptions=["-9", "-l"],
            initrdmodules=["dm-crypt", "slimbus"],
            initrdstagefirmware=True,
            initrdfirmware=["firmware/for/wifi", "firmware/for/webcam"],
            initrdaddons=[
                "usr/bin/cryptsetup",
                "usr/lib/my-arch/libcrypto.so",
            ],
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

    def test_check_get_build_command_comp_modules_modules_configured_overlay(self):
        plugin = self._setup_test(
            initrdmodules=["dm-crypt", "slimbus"],
            initrdconfiguredmodules=["libarc4"],
            initrdoverlay="my-overlay",
            initrdcompression="gz",
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

    def test_check_get_build_command_modules_modules_configured_overlay_cross(self):
        plugin = self._setup_test(
            initrdmodules=["dm-crypt", "slimbus"],
            initrdconfiguredmodules=["libarc4"],
            initrdoverlay="my-overlay",
            arch="armv7l",
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
    "riscv64": "riscv64",
    "x86_64": "amd64",
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
        for m in $(cat ${initramfs_ko_modules_conf})
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
        sed -i 's/lz4 -9 -l/lz4 -9 -l/g' "${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs"',
        """
    )
]

_update_initrd_compression_gz_cmd = [
    textwrap.dedent(
        """
        echo "Updating compression command to be used for initrd"
        sed -i 's/lz4 -9 -l/gzip -7/g' "${UC_INITRD_ROOT}/usr/bin/ubuntu-core-initramfs"',
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

        link_files "${UC_INITRD_ROOT}"root "kernel.efi*" "${CRAFT_PART_INSTALL}"
        ln -f "$(ls "${CRAFT_PART_INSTALL}"/kernel.efi*)" "${CRAFT_PART_INSTALL}/kernel.efi
        rm "${CRAFT_PART_INSTALL}"/initrd.img*
        """
    )
]
