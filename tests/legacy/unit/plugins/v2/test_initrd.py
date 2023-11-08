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

import platform
import textwrap
from dataclasses import dataclass
from unittest import mock

from testtools import TestCase

from snapcraft_legacy.plugins.v2.initrd import InitrdPlugin


class Initrdv2PluginProperties:
    initrd_modules: [str] = None
    initrd_configured_modules: [str] = None
    initrd_firmware: [str] = None
    initrd_compression: [str] = None
    initrd_compression_options: [str] = None
    initrd_overlay: str = None
    initrd_addons: [str] = None
    initrd_add_ppa: bool = False


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
            # Ensure that the PPA is not added so we don't cause side-effects
            initrd_add_ppa = False

        target_arch = _DEB_ARCH_TRANSLATIONS[arch]

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
                    "initrd-stage-firmware": {
                        "type": "boolean",
                        "default": False,
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
                    "initrd-add-ppa": {
                        "type": "boolean",
                        "default": True,
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
                "fakeroot",
                "dracut-core",
                "kmod",
                "kpartx",
                "lz4",
                "systemd",
            },
        )

    def test_get_base_build_packages_lz4(self):
        plugin = self._setup_test(initrdcompression="lz4")
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "bc",
                "binutils",
                "fakeroot",
                "dracut-core",
                "kmod",
                "kpartx",
                "lz4",
                "systemd",
            },
        )

    def test_get_base_build_packages_xz(self):
        plugin = self._setup_test(initrdcompression="xz")
        self.assertEqual(
            plugin.get_build_packages(),
            {
                "bc",
                "binutils",
                "fakeroot",
                "dracut-core",
                "kmod",
                "kpartx",
                "xz-utils",
                "systemd",
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
        self.assertFalse(opt.initrd_add_ppa)

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
        self.assertFalse(opt.initrd_add_ppa)

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
        self.assertFalse(opt.initrd_add_ppa)

    def test_check_get_build_environment(self):
        plugin = self._setup_test()

        self.assertEqual(
            plugin.get_build_environment(),
            {
                "UC_INITRD_DEB": "${SNAPCRAFT_PART_BUILD}/ubuntu-core-initramfs",
            },
        )

    def test_check_get_build_environment_compiler_paths(self):
        plugin = self._setup_test()

        self.assertEqual(
            plugin.get_build_environment(),
            {
                "UC_INITRD_DEB": "${SNAPCRAFT_PART_BUILD}/ubuntu-core-initramfs",
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
        assert _is_sub_array(build_commands, _download_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
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

    def test_check_get_build_command_defconfig_configs_no_firmware_lz4(self):
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
        assert _is_sub_array(build_commands, _download_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
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

    def test_check_get_build_command_unknown_compiler(self):
        plugin = self._setup_test()

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _initrd_modules_empty_cmd)
        assert _is_sub_array(build_commands, _initrd_configured_modules_empty_cmd)
        assert _is_sub_array(build_commands, _link_files_fnc)
        assert _is_sub_array(build_commands, _download_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
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

    def test_check_get_build_command_config_flavour_configs(self):
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
        assert _is_sub_array(build_commands, _download_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
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

    def test_check_get_build_command_cross(self):
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
        assert _is_sub_array(build_commands, _download_initrd_fnc)
        assert _is_sub_array(build_commands, _get_initrd_armhf_cmd)
        assert _is_sub_array(build_commands, _download_snapd_fnc)
        assert _is_sub_array(build_commands, _get_snapd_armhf_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
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


_initrd_modules_empty_cmd = ['initrd_installed_kernel_modules=""']

_initrd_modules_cmd = ['initrd_installed_kernel_modules="dm-crypt slimbus"']

_initrd_configured_modules_empty_cmd = ['initrd_configured_kernel_modules=""']

_initrd_configured_modules_cmd = ['initrd_configured_kernel_modules="libarc4"']

_link_files_fnc = [
    textwrap.dedent(
        """
        # link files, accept wild cards
        # 1: reference dir, 2: file(s) including wild cards, 3: dst dir
        link_files() {
            if [ "${2}" = "*" ]; then
                for f in $(ls ${1})
                do
                    link_files "${1}" "${f}" "${3}"
                done
                return 0
            fi
            if [ -d "${1}/${2}" ]; then
                for f in $(ls ${1}/${2})
                do
                    link_files "${1}" "${2}/${f}" "${3}"
                done
                return 0
            fi

            local found=""
            for f in $(ls ${1}/${2})
            do
                if [[ -L "${f}" ]]; then
                    local rel_path=$( realpath --no-symlinks --relative-to=${1} ${f} )
                else
                    local rel_path=$( realpath -se --relative-to=${1} ${f} )
                fi
                local dir_path=$(dirname ${rel_path})
                mkdir -p ${3}/${dir_path}
                echo "installing ${f} to ${3}/${dir_path}"
                ln -f ${f} ${3}/${dir_path}
                found="yes"
            done
            if [ "yes" = "${found}" ]; then
                return 0
            else
                return 1
            fi
        }
        """
    )
]

_parts_source_dir = "${SNAPCRAFT_PART_SRC}"

_download_initrd_fnc = [
    textwrap.dedent(
        """
        # Helper to download code initrd deb package
        # 1: arch, 2: output dir 3: source dir
        download_core_initrd() {
            # skip download if file already exist
            if ! ls ${3}/ubuntu-core-initramfs_*.deb 1> /dev/null 2>&1; then
                apt-get download ubuntu-core-initramfs:${1}
                mv ubuntu-core-initramfs_*.deb ${3}
            fi
            # unpack dep to the target dir
            dpkg -x ${3}/ubuntu-core-initramfs_*.deb ${2}
        }
        """
    )
]

_get_initrd_cmd = [
    textwrap.dedent(
        """
        echo "Getting ubuntu-core-initrd...."
        # only download u-c-initrd deb if needed
        if [ ! -e ${{UC_INITRD_DEB}} ]; then
            download_core_initrd {arch} ${{UC_INITRD_DEB}} {parts_source_dir}
        fi
        """.format(
            arch=_DEB_ARCH_TRANSLATIONS[platform.machine()],
            parts_source_dir=_parts_source_dir,
        )
    )
]


_get_initrd_armhf_cmd = [
    textwrap.dedent(
        """
        echo "Getting ubuntu-core-initrd...."
        # only download u-c-initrd deb if needed
        if [ ! -e ${{UC_INITRD_DEB}} ]; then
            download_core_initrd {arch} ${{UC_INITRD_DEB}} {parts_source_dir}
        fi
        """.format(
            arch="armhf", parts_source_dir=_parts_source_dir
        )
    )
]

_download_snapd_fnc = [
    textwrap.dedent(
        """
        # Helper to download snap-bootstrap from snapd deb package
        # 1: arch, 2: output dir 3: source dir
        download_snap_bootstrap() {
            # skip download if file already exist
            if ! ls ${3}/snapd_*.deb 1> /dev/null 2>&1; then
                apt-get download snapd:${1}
                mv snapd_*.deb ${3}
            fi
            # unpack dep to the target dir
            dpkg -x ${3}/snapd_*.deb ${2}
        }
        """
    )
]

_get_snapd_cmd = [
    textwrap.dedent(
        """
        echo "Getting snapd deb for snap bootstrap..."
        # only download again if files does not exist, otherwise
        # assume we are re-running build
        if [ ! -e ${{UC_INITRD_DEB}}/usr/lib/snapd ]; then
            download_snap_bootstrap {arch} ${{UC_INITRD_DEB}} {parts_source_dir}
        fi
        """.format(
            arch=_DEB_ARCH_TRANSLATIONS[platform.machine()],
            parts_source_dir=_parts_source_dir,
        )
    )
]

_get_snapd_armhf_cmd = [
    textwrap.dedent(
        """
        echo "Getting snapd deb for snap bootstrap..."
        # only download again if files does not exist, otherwise
        # assume we are re-running build
        if [ ! -e ${{UC_INITRD_DEB}}/usr/lib/snapd ]; then
            download_snap_bootstrap {arch} ${{UC_INITRD_DEB}} {parts_source_dir}
        fi
        """.format(
            arch="armhf", parts_source_dir=_parts_source_dir
        )
    )
]

_parse_kernel_release_cmd = [
    "KERNEL_RELEASE=$(ls ${SNAPCRAFT_STAGE}/modules)",
]

_install_initrd_modules_cmd = [
    'echo "Installing ko modules to initrd..."',
    'install_modules=""',
    'echo "Gathering module dependencies..."',
    'install_modules=""',
    "uc_initrd_feature_kernel_modules=${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/kernel-modules",
    "mkdir -p ${uc_initrd_feature_kernel_modules}",
    "initramfs_ko_modules_conf=${uc_initrd_feature_kernel_modules}/extra-modules.conf",
    "touch ${initramfs_ko_modules_conf}",
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
    'echo "If module does not exist, do not include it"',
    "initramfs_conf_dir=${uc_initrd_feature_kernel_modules}/usr/lib/modules-load.d",
    "mkdir -p ${initramfs_conf_dir}",
    "initramfs_conf=${initramfs_conf_dir}/ubuntu-core-initramfs.conf",
    'echo "# configured modules" > ${initramfs_conf}',
    "for m in $(cat ${initramfs_ko_modules_conf})",
    "do",
    " ".join(
        [
            "\tif",
            "[ -n",
            '"$(modprobe -n -q --show-depends',
            "-d ${SNAPCRAFT_STAGE}",
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
            '"${UC_INITRD_DEB}" "usr/lib/snapd/snap-bootstrap"',
            '"${uc_initrd_feature_snap_bootstratp}"',
        ],
    ),
    'link_files "${UC_INITRD_DEB}" "usr/lib/snapd/info" "${uc_initrd_feature_snap_bootstratp}"',
    "cp ${UC_INITRD_DEB}/usr/lib/snapd/info ${SNAPCRAFT_PART_INSTALL}/snapd-info",
]

_clean_old_initrd_cmd = [
    "if compgen -G ${SNAPCRAFT_PART_INSTALL}/initrd.img* > /dev/null; then",
    "\trm -rf ${SNAPCRAFT_PART_INSTALL}/initrd.img*",
    "fi",
]

_initrd_check_firmware_stage = [
    '[ ! -d "${SNAPCRAFT_STAGE}/firmware" ] && echo -e "firmware directory '
    "${SNAPCRAFT_STAGE}/firmware does not exist, consider using "
    'initrd-stage-firmware: true/false option" && exit 1'
]

_initrd_check_firmware_part = [
    '[ ! -d "${SNAPCRAFT_PART_INSTALL}/lib/firmware" ] && echo -e "firmware directory '
    "${SNAPCRAFT_PART_INSTALL}/lib/firmware does not exist, consider using "
    'initrd-stage-firmware: true/false option" && exit 1'
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
            "--kerneldir ${SNAPCRAFT_STAGE}/modules/${KERNEL_RELEASE}",
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
            "--kerneldir ${SNAPCRAFT_STAGE}/modules/${KERNEL_RELEASE}",
            "--firmwaredir ${SNAPCRAFT_STAGE}/firmware",
            "--skeleton ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs",
            "--output ${SNAPCRAFT_PART_INSTALL}/initrd.img",
        ],
    ),
    "ln $(ls ${SNAPCRAFT_PART_INSTALL}/initrd.img*) ${SNAPCRAFT_PART_INSTALL}/initrd.img",
]
