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

import logging
import platform
import subprocess
import textwrap

import pytest
from craft_parts import Part, PartInfo, ProjectInfo
from pydantic import ValidationError

from snapcraft.parts.plugins import InitrdPlugin


@pytest.fixture
def setup_method_fixture():
    def _setup_method_fixture(
        new_dir, properties=None, arch=None, initrd_add_ppa=False
    ):
        if properties is None:
            properties = {}
        # Ensure that the PPA is not added so we don't cause side-effects
        properties["initrd-add-ppa"] = initrd_add_ppa

        part = Part("initrd", {})

        if arch is None:
            project_info = ProjectInfo(application_name="test", cache_dir=new_dir)
        else:
            project_info = ProjectInfo(
                application_name="test", cache_dir=new_dir, arch=arch
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
            "bc",
            "binutils",
            "fakeroot",
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
                "initrd-compression": "lz4",
            },
        )
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "fakeroot",
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
                "initrd-compression": "xz",
            },
        )
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "fakeroot",
            "dracut-core",
            "kmod",
            "kpartx",
            "xz-utils",
            "systemd",
        }

    def test_get_base_build_packages_efi(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-build-efi-image": "true",
            },
        )
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "fakeroot",
            "dracut-core",
            "kmod",
            "kpartx",
            "llvm",
            "zstd",
            "systemd",
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
        assert opt.initrd_modules is None
        assert opt.initrd_configured_modules is None
        assert opt.initrd_firmware is None
        assert opt.initrd_compression is None
        assert opt.initrd_compression_options is None
        assert opt.initrd_overlay is None
        assert opt.initrd_addons is None
        assert not opt.initrd_add_ppa

    def test_check_configuration_zstd_compression(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-compression": "zstd",
            },
        )
        opt = plugin.options

        assert not opt.initrd_build_efi_image
        assert opt.initrd_modules is None
        assert opt.initrd_configured_modules is None
        assert opt.initrd_firmware is None
        assert opt.initrd_compression == "zstd"
        assert opt.initrd_compression_options is None
        assert opt.initrd_overlay is None
        assert opt.initrd_addons is None
        assert not opt.initrd_add_ppa

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
        assert opt.initrd_modules is None
        assert opt.initrd_configured_modules is None
        assert opt.initrd_firmware is None
        assert opt.initrd_compression == "lz4"
        assert opt.initrd_compression_options == ["-9", "-l"]
        assert opt.initrd_overlay is None
        assert opt.initrd_addons is None
        assert not opt.initrd_add_ppa

    def test_check_configuration_image_missing_initrd_compression(
        self, setup_method_fixture, new_dir
    ):
        try:
            setup_method_fixture(
                new_dir,
                properties={
                    "initrd-compression-options": {"-9", "-l"},
                },
            )
        except ValidationError as err:
            error_list = err.errors()
            assert len(error_list) == 1
            error = error_list[0]
            assert "initrd-compression-options requires" in error.get("msg")

    def test_check_out_of_source_build(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        assert plugin.get_out_of_source_build() is True

    def test_check_get_build_environment(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

        assert plugin.get_build_environment() == {
            "UC_INITRD_DEB": "${CRAFT_PART_BUILD}/ubuntu-core-initramfs",
        }

    def test_check_get_build_command(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

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

    def test_check_get_build_command_defconfig_configs_no_firmware_lz4(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-compression": "lz4",
                "initrd-compression-options": ["-9", "-l"],
                "initrd-modules": ["dm-crypt", "slimbus"],
                "initrd-stage-firmware": True,
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

    def test_check_get_build_command_unknown_compiler(
        self, setup_method_fixture, new_dir, caplog
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={},
        )

        # we need to get build environment
        plugin.get_build_environment()
        with caplog.at_level(logging.INFO):
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

    def test_check_get_build_command_config_flavour_configs(
        self, setup_method_fixture, new_dir
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-modules": ["dm-crypt", "slimbus"],
                "initrd-configured-modules": ["libarc4"],
                "initrd-overlay": "my-overlay",
                "initrd-compression": "gz",
                "initrd-build-efi-image": "true",
            },
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
        assert _is_sub_array(build_commands, _create_efi_image_cmd)

    def test_check_get_build_command_cross(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "initrd-modules": ["dm-crypt", "slimbus"],
                "initrd-configured-modules": ["libarc4"],
                "initrd-overlay": "my-overlay",
            },
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

    @pytest.mark.parametrize("arch", ["aarch64", "armv7l", "riscv64", "x86_64"])
    def test_check_arch(self, arch, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir, arch=arch)

        assert plugin._target_arch == _DEB_ARCH_TRANSLATIONS[arch]

    def test_add_ppa_key_exists(self, setup_method_fixture, new_dir, fp):
        # we need to  mock subprocess.run first
        fp.register(["apt-get", "install", "-y", "software-properties-common"])
        fp.register(
            ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"],
            stdout=[
                "/etc/apt/sources.list.d/snappy-dev-ubuntu-image-focal.list:deb"
                "http://ppa.launchpad.net/snappy-dev/image/ubuntu",
                "focal",
                "main",
            ],
        )
        plugin = setup_method_fixture(new_dir, initrd_add_ppa=True)
        plugin.get_build_packages()
        assert (
            fp.call_count(
                ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"]
            )
            == 1
        )

    def test_add_ppa_key_exists_happy(self, setup_method_fixture, new_dir, caplog, fp):
        # we need to  mock subprocess.run first
        fp.register(["apt-get", "install", "-y", "software-properties-common"])
        fp.register(
            ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"],
            stdout=[""],
        )
        fp.register(
            ["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT],
            stdout=[
                "----BEGIN PGP PUBLIC KEY BLOCK-----",
                "",
                "mQINBFRt70cBEADH/8JgKzFnwQQqtllZ3nqxYQ1cZguLCbyu9s1AwRDNu0P2oWOR",
                "UN9YoUS15kuWtTuneVlLbdbda3N/S/HApvOWu7Q1oIrRRkpO4Jv4xN+1KaSpaTy1",
            ],
        )
        fp.register(["add-apt-repository", "-y", "ppa:snappy-dev/image"], stdout=[""])

        plugin = setup_method_fixture(new_dir, initrd_add_ppa=True)
        with caplog.at_level(logging.INFO):
            plugin.get_build_packages()
        assert (
            fp.call_count(
                ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"]
            )
            == 1
        )
        assert fp.call_count(["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT]) == 1
        assert "key for ppa:snappy-dev/image already imported" in caplog.text
        assert "adding ppa:snappy-dev/image to handle initrd builds" in caplog.text
        assert fp.call_count(["add-apt-repository", "-y", "ppa:snappy-dev/image"]) == 1

    def test_add_ppa_no_key_happy(self, setup_method_fixture, new_dir, caplog, fp):
        # we need to  mock subprocess.run first
        fp.register(["apt-get", "install", "-y", "software-properties-common"])
        fp.register(
            ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"],
            stdout=[""],
        )
        fp.register(
            ["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT],
            stdout=["gpg: WARNING: nothing exported"],
        )
        fp.register(
            [
                "apt-key",
                "adv",
                "--keyserver",
                "keyserver.ubuntu.com",
                "--recv-keys",
                _SNAPPY_DEV_KEY_FINGERPRINT,
            ],
            stdout=[""],
        )
        fp.register(["add-apt-repository", "-y", "ppa:snappy-dev/image"], stdout=[""])

        plugin = setup_method_fixture(new_dir, initrd_add_ppa=True)
        with caplog.at_level(logging.INFO):
            plugin.get_build_packages()
        assert (
            fp.call_count(
                ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"]
            )
            == 1
        )
        assert fp.call_count(["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT]) == 1
        assert "importing key for ppa:snappy-dev/image" in caplog.text
        assert (
            fp.call_count(
                [
                    "apt-key",
                    "adv",
                    "--keyserver",
                    "keyserver.ubuntu.com",
                    "--recv-keys",
                    _SNAPPY_DEV_KEY_FINGERPRINT,
                ],
            )
            == 1
        )
        assert "adding ppa:snappy-dev/image to handle initrd builds" in caplog.text
        assert fp.call_count(["add-apt-repository", "-y", "ppa:snappy-dev/image"]) == 1

    def test_add_ppa_key_check_unhappy(self, setup_method_fixture, new_dir, caplog, fp):
        # we need to  mock subprocess.run first
        fp.register(["apt-get", "install", "-y", "software-properties-common"])
        fp.register(
            ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"],
            stdout=[""],
        )
        fp.register(
            ["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT],
            callback=subprocess_callback_function,
        )
        fp.register(["add-apt-repository", "-y", "ppa:snappy-dev/image"], stdout=[""])

        plugin = setup_method_fixture(new_dir, initrd_add_ppa=True)
        with caplog.at_level(logging.INFO):
            try:
                plugin.get_build_packages()
            except ValueError as e:
                assert (
                    str(e) == "error to check for key=F1831DDAFC42E99D: command failed"
                )

        assert (
            fp.call_count(
                ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"]
            )
            == 1
        )
        assert fp.call_count(["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT]) == 1
        assert fp.call_count(["add-apt-repository", "-y", "ppa:snappy-dev/image"]) == 0

    def test_add_ppa_key_add_unhappy(self, setup_method_fixture, new_dir, caplog, fp):
        # we need to  mock subprocess.run first
        fp.register(["apt-get", "install", "-y", "software-properties-common"])
        fp.register(
            ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"],
            stdout=[""],
        )
        fp.register(
            ["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT],
            stdout=["gpg: WARNING: nothing exported"],
        )
        fp.register(
            [
                "apt-key",
                "adv",
                "--keyserver",
                "keyserver.ubuntu.com",
                "--recv-keys",
                _SNAPPY_DEV_KEY_FINGERPRINT,
            ],
            callback=subprocess_callback_function,
        )
        fp.register(["add-apt-repository", "-y", "ppa:snappy-dev/image"], stdout=[""])

        plugin = setup_method_fixture(new_dir, initrd_add_ppa=True)
        with caplog.at_level(logging.INFO):
            try:
                plugin.get_build_packages()
            except ValueError as e:
                assert (
                    str(e) == "Failed to add ppa key: F1831DDAFC42E99D: command failed"
                )

        assert (
            fp.call_count(
                ["grep", "-r", "snappy-dev/image/ubuntu", "/etc/apt/sources.list.d/"]
            )
            == 1
        )
        assert fp.call_count(["apt-key", "export", _SNAPPY_DEV_KEY_FINGERPRINT]) == 1
        assert (
            fp.call_count(
                [
                    "apt-key",
                    "adv",
                    "--keyserver",
                    "keyserver.ubuntu.com",
                    "--recv-keys",
                    _SNAPPY_DEV_KEY_FINGERPRINT,
                ]
            )
            == 1
        )
        assert fp.call_count(["add-apt-repository", "-y", "ppa:snappy-dev/image"]) == 0


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
    "aarch64": "arm64",
    "armv7l": "armhf",
    "riscv64": "riscv64",
    "x86_64": "amd64",
}

_SNAPPY_DEV_KEY_FINGERPRINT = "F1831DDAFC42E99D"

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

_download_initrd_fnc = [
    textwrap.dedent(
        """
        # Helper to download code initrd deb package
        # 1: arch, 2: output dir
        download_core_initrd() {
        	apt-get download ubuntu-core-initramfs:${1}
        	# unpack dep to the target dir
        	dpkg -x ubuntu-core-initramfs_*.deb ${2}
        }
        """
    )
]

_machine_arch = _DEB_ARCH_TRANSLATIONS[platform.machine()]

_get_initrd_cmd = [
    textwrap.dedent(
        f"""
        echo "Getting ubuntu-core-initrd...."
        # only download u-c-initrd deb if needed
        if [ ! -e ${{UC_INITRD_DEB}} ]; then
        	download_core_initrd {_machine_arch} ${{UC_INITRD_DEB}}
        fi
        """
    )
]

_get_initrd_armhf_cmd = [
    textwrap.dedent(
        """
        echo "Getting ubuntu-core-initrd...."
        # only download u-c-initrd deb if needed
        if [ ! -e ${UC_INITRD_DEB} ]; then
        	download_core_initrd armhf ${UC_INITRD_DEB}
        fi
        """
    )
]

_download_snapd_fnc = [
    textwrap.dedent(
        """
        # Helper to download snap-bootstrap from snapd deb package
        # 1: arch, 2: output dir
        download_snap_bootstrap() {
        	apt-get download snapd:${1}
        	# unpack dep to the target dir
        	dpkg -x snapd_*.deb ${2}
        }
        """
    )
]

_get_snapd_cmd = [
    textwrap.dedent(
        f"""
        echo "Getting snapd deb for snap bootstrap..."
        # only download again if files does not exist, otherwise
        # assume we are re-running build
        if [ ! -e ${{UC_INITRD_DEB}}/usr/lib/snapd ]; then
        	download_snap_bootstrap {_machine_arch} ${{UC_INITRD_DEB}}
        fi
        """
    )
]

_get_snapd_armhf_cmd = [
    textwrap.dedent(
        """
        echo "Getting snapd deb for snap bootstrap..."
        # only download again if files does not exist, otherwise
        # assume we are re-running build
        if [ ! -e ${UC_INITRD_DEB}/usr/lib/snapd ]; then
        	download_snap_bootstrap armhf ${UC_INITRD_DEB}
        fi
        """
    )
]


_parse_kernel_release_cmd = [
    "KERNEL_RELEASE=$(ls ${CRAFT_STAGE}/modules)",
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
    "for m in ${initrd_configured_kernel_modules}",
    "do",
    " ".join(
        [
            "\tif",
            "[ -n",
            '"$(modprobe -n -q --show-depends',
            "-d ${CRAFT_STAGE}",
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
    '\tif ! link_files "${CRAFT_PART_INSTALL}" "${f}" "${uc_initrd_feature_firmware}/lib" ; then',
    '\t\tif ! link_files "${CRAFT_STAGE}" "${f}" "${uc_initrd_feature_firmware}/lib" ; then',
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
    '\tlink_files "${CRAFT_STAGE}" "${a}" "${uc_initrd_feature_overlay}"',
    "done",
]

_intatll_initrd_overlay_cmd = [
    'link_files "${CRAFT_STAGE}/my-overlay" "" "${uc_initrd_feature_overlay}"'
]

_prepare_ininird_features_cmd = [
    'echo "Preparing snap-boostrap initrd feature..."',
    "uc_initrd_feature_snap_bootstratp="
    "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snap-bootstrap",
    "mkdir -p ${uc_initrd_feature_snap_bootstratp}",
    " ".join(
        [
            "link_files",
            '"${UC_INITRD_DEB}" "usr/lib/snapd/snap-bootstrap"',
            '"${uc_initrd_feature_snap_bootstratp}"',
        ],
    ),
    'link_files "${UC_INITRD_DEB}" "usr/lib/snapd/info"'
    ' "${uc_initrd_feature_snap_bootstratp}"',
    "cp ${UC_INITRD_DEB}/usr/lib/snapd/info ${CRAFT_PART_INSTALL}/snapd-info",
]

_clean_old_initrd_cmd = [
    "if compgen -G ${CRAFT_PART_INSTALL}/initrd.img* > /dev/null; then",
    "\trm -rf ${CRAFT_PART_INSTALL}/initrd.img*",
    "fi",
]

_initrd_check_firmware_stage = [
    '[ ! -d "${CRAFT_STAGE}/firmware" ] && echo -e "firmware directory '
    "${CRAFT_STAGE}/firmware does not exist, consider using "
    'initrd-stage-firmware: true/false option" && exit 1'
]

_initrd_check_firmware_part = [
    '[ ! -d "${CRAFT_PART_INSTALL}/lib/firmware" ] && echo -e "firmware directory '
    "${CRAFT_PART_INSTALL}/lib/firmware does not exist, consider using "
    'initrd-stage-firmware: true/false option" && exit 1'
]

_initrd_tool_cmd = [
    "ubuntu_core_initramfs=${UC_INITRD_DEB}/usr/bin/ubuntu-core-initramfs",
]

_update_initrd_compression_cmd = [
    'echo "Updating compression command to be used for initrd"',
    "sed -i 's/zstd -1 -T0/lz4 -9 -l/g' ${ubuntu_core_initramfs}",
]

_update_initrd_compression_gz_cmd = [
    'echo "Updating compression command to be used for initrd"',
    "sed -i 's/zstd -1 -T0/gzip -7/g' ${ubuntu_core_initramfs}",
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
    " ".join(
        [
            "cp",
            "${initramfs_ko_modules_conf}",
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
            "--kerneldir ${CRAFT_STAGE}/modules/${KERNEL_RELEASE}",
            "--firmwaredir ${CRAFT_PART_INSTALL}/lib/firmware",
            "--skeleton ${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs",
            "--output ${CRAFT_PART_INSTALL}/initrd.img",
        ],
    ),
    "ln $(ls ${CRAFT_PART_INSTALL}/initrd.img*) ${CRAFT_PART_INSTALL}/initrd.img",
]

_create_inird_stage_firmware_cmd = [
    " ".join(
        [
            "${ubuntu_core_initramfs}",
            "create-initrd",
            "--kernelver=${KERNEL_RELEASE}",
            "--kerneldir ${CRAFT_STAGE}/modules/${KERNEL_RELEASE}",
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
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.key",
            "--cert",
            "${UC_INITRD_DEB}/usr/lib/ubuntu-core-initramfs/snakeoil/PkKek-1-snakeoil.pem",
            "--initrd",
            "${CRAFT_PART_INSTALL}/initrd.img",
            "--kernel",
            "${CRAFT_PART_INSTALL}/${KERNEL_IMAGE_TARGET}",
            "--output",
            "${CRAFT_PART_INSTALL}/kernel.efi",
        ],
    ),
    "ln $(ls ${CRAFT_PART_INSTALL}/kernel.efi*) ${CRAFT_PART_INSTALL}/kernel.efi",
]
