# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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
import os
import platform
import subprocess
import sys
import tempfile
import textwrap

import pytest
from craft_parts import Part, PartInfo, ProjectInfo
from craft_parts.errors import InvalidArchitecture
from craft_platforms import DebianArchitecture
from pydantic import ValidationError

from snapcraft.parts.plugins import KernelPlugin
from snapcraft_legacy.plugins.v2._kernel_build import check_new_config


@pytest.fixture
def setup_method_fixture():
    def _setup_method_fixture(new_dir, properties=None, arch=None):
        if properties is None:
            properties = {}

        part = Part("kernel", {})

        if arch is None:
            project_info = ProjectInfo(application_name="test", cache_dir=new_dir)
        else:
            project_info = ProjectInfo(
                application_name="test", cache_dir=new_dir, arch=arch
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
            "debhelper",
            "fakeroot",
            "gcc",
            "cmake",
            "cryptsetup",
            "kmod",
            "kpartx",
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
        }

    def test_get_base_build_packages_zfs_cross(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-enable-zfs-support": True,
            },
            arch="armhf",
        )
        assert plugin.get_build_packages() == {
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
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf

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
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf

    def test_check_configuration_image_target(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kdefconfig": ["snappy_defconfig"],
                "kernel-image-target": {"arm64": "Image", "armhf": "Image.gz"},
                "kernel-with-firmware": True,
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
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf

    def test_check_configuration_konfig_file(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-kconfigfile": "arch/arm64/configs/snappy_defconfig",
                "kernel-with-firmware": True,
                "kernel-kconfigs": ["CONFIG_DEBUG_INFO=n", "CONFIG_DM_CRYPT=y"],
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
        assert opt.kernel_compiler is None
        assert opt.kernel_compiler_paths is None
        assert opt.kernel_compiler_parameters is None
        assert not opt.kernel_enable_zfs_support
        assert not opt.kernel_enable_perf

    def test_check_configuration_image_wrong_image_target(
        self, setup_method_fixture, new_dir
    ):
        try:
            setup_method_fixture(
                new_dir,
                properties={
                    "kernel-image-target": {"arm64", "Image", "armhf", "Image.gz"},
                },
            )
        except ValidationError as err:
            error_list = err.errors()
            assert len(error_list) == 2
            assert error_list[0].get("loc") == ("kernel-image-target", "str")
            assert error_list[0].get("msg") == "Input should be a valid string"
            assert error_list[1].get("loc") == ("kernel-image-target", "dict[str,any]")
            assert error_list[1].get("msg") == "Input should be a valid dictionary"

    def test_check_out_of_source_build(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        assert plugin.get_out_of_source_build() is True

    def test_check_get_build_environment(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)
        plugin._kernel_arch = "amd64"

        assert plugin.get_build_environment() == {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET}-",
            "ARCH": plugin._kernel_arch,
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
            "KERNEL_BUILD_ARCH_DIR": f"${{CRAFT_PART_BUILD}}/arch/{plugin._kernel_arch}/boot",
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
            arch="armhf",
        )

        assert plugin.get_build_environment() == {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET}-",
            "ARCH": "arm",
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
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
        plugin._kernel_arch = "amd64"

        assert plugin.get_build_environment() == {
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET}-",
            "ARCH": plugin._kernel_arch,
            "DEB_ARCH": "${CRAFT_TARGET_ARCH}",
            "KERNEL_BUILD_ARCH_DIR": f"${{CRAFT_PART_BUILD}}/arch/{plugin._kernel_arch}/boot",
            "KERNEL_IMAGE_TARGET": plugin.kernel_image_target,
            "PATH": "${CRAFT_STAGE}/gcc-11/bin:${CRAFT_STAGE}/gcc-11/sbin:${PATH}",
        }

    def test_check_get_build_command(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(new_dir)

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
            },
        )

        # we need to get build environment
        plugin.get_build_environment()
        build_commands = plugin.get_build_commands()
        assert _is_sub_array(build_commands, _determine_kernel_src)
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
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert not _is_sub_array(build_commands, _build_zfs_cmd)
        assert not _is_sub_array(build_commands, _build_perf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    def test_check_get_build_command_unknown_compiler(
        self, setup_method_fixture, new_dir, caplog
    ):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-compiler": "my-gcc",
            },
        )

        # we need to get build environment
        plugin.get_build_environment()
        with caplog.at_level(logging.INFO):
            build_commands = plugin.get_build_commands()

        assert "Only other 'supported' compiler is clang" in caplog.text
        assert "hopefully you know what you are doing" in caplog.text
        assert _is_sub_array(build_commands, _determine_kernel_src)
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
                "kernel-device-trees": ["pi3", "pi3b", "pi4", "pi/cm3", "pi/cm4"],
                "kernel-with-firmware": False,
                "kernel-enable-zfs-support": True,
                "kernel-enable-perf": True,
            },
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
            assert _is_sub_array(build_commands, _build_kernel_dtbs_x86_cmd)
        else:
            assert _is_sub_array(build_commands, _build_kernel_dtbs_cmd)

        assert _is_sub_array(build_commands, _install_kernel_no_dtbs_no_firmware_cmd)
        assert _is_sub_array(build_commands, _parse_kernel_release_cmd)
        assert _is_sub_array(build_commands, _install_dtbs_cmd)
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
            },
            arch="armhf",
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
        assert not _is_sub_array(build_commands, _install_dtbs_cmd)
        assert _is_sub_array(build_commands, _install_config_cmd)
        assert _is_sub_array(build_commands, _build_zfs_cmd)
        assert _is_sub_array(build_commands, _build_perf_armhf_cmd)
        assert _is_sub_array(build_commands, _finalize_install_cmd)

    @pytest.mark.parametrize("arch", ["arm64", "armhf", "riscv64", "amd64"])
    def test_check_arch(self, arch, setup_method_fixture, new_dir):
        cross_building = DebianArchitecture.from_host() != arch
        plugin = setup_method_fixture(new_dir, arch=arch)

        assert plugin._kernel_arch == _DEB_TO_KERNEL_ARCH_TRANSLATIONS[arch]
        assert plugin._deb_arch == arch
        assert plugin._target_arch == arch
        assert plugin._cross_building == cross_building

    def test_check_arch_i686(self, setup_method_fixture, new_dir):
        # we do not support i686 so use this arch to test unknown arch by plugin
        error = "Architecture 'i686' is not supported"
        with pytest.raises(InvalidArchitecture, match=error):
            setup_method_fixture(new_dir, arch="i686")

    def test_check_new_config_good(self, setup_method_fixture, new_dir, caplog):
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
            with caplog.at_level(logging.WARNING):
                check_new_config(config_path=config_file.name, initrd_modules=[])
            assert caplog.text == ""

    def test_check_new_config_missing(self, setup_method_fixture, new_dir, caplog):
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
            with caplog.at_level(logging.WARNING):
                check_new_config(config_path=config_file.name, initrd_modules=[])
            logs = list(
                filter(
                    None,
                    caplog.text.split(
                        "WARNING  snapcraft_legacy.plugins.v2._kernel_build:_kernel_build.py"
                    ),  # XXX: use a better way to check log messages
                )
            )
            # there should be 2 warning logs, one for missing configs, one for kernel module
            assert len(logs) == 2
            assert "**** WARNING **** WARNING **** WARNING **** WARNING ****" in logs[0]
            assert "CONFIG_SECCOMP" in logs[0]
            assert "CONFIG_SQUASHFS" in logs[0]
            assert (
                "CONFIG_SQUASHFS_LZO (used by desktop snaps for accelerated loading)"
                in logs[0]
            )
            assert "**** WARNING **** WARNING **** WARNING **** WARNING ****" in logs[1]
            assert "adding\nthe corresponding module to initrd:" in logs[1]
            assert "CONFIG_SQUASHFS" in logs[1]

    def test_check_new_config_squash_module_missing(
        self, setup_method_fixture, new_dir, caplog
    ):
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
            with caplog.at_level(logging.WARNING):
                check_new_config(config_path=config_file.name, initrd_modules=[])
            # there should be 1 warning log for missing module in initrd
            logs = list(
                filter(
                    None,
                    caplog.text.split(
                        "WARNING  snapcraft.parts.plugins.kernel:kernel.py"
                    ),
                )
            )
            assert len(logs) == 1
            assert "**** WARNING **** WARNING **** WARNING **** WARNING ****" in logs[0]
            assert "adding\nthe corresponding module to initrd:" in logs[0]
            assert "CONFIG_SQUASHFS" in logs[0]

    def test_check_new_config_squash_module(
        self, setup_method_fixture, new_dir, caplog
    ):
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
            with caplog.at_level(logging.WARNING):
                check_new_config(
                    config_path=config_file.name, initrd_modules=["squashfs"]
                )

            # there should be no warnings
            assert caplog.text == ""

    def test_external_check_new_config(self, setup_method_fixture):
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
            plugin_path = os.path.abspath(check_new_config.__globals__["__file__"])
            proc = subprocess.run(
                [
                    sys.executable,
                    "-I",
                    plugin_path,
                    "check_new_config",
                    config_file.name,
                    "squashfs",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )

            out = proc.stdout.decode()
            assert out == "Checking created config...\n"

    def test_check_new_config_squash_missing_file(self, setup_method_fixture, new_dir):
        # run with invalid file
        e = ""
        try:
            check_new_config(config_path="wrong/file", initrd_modules=["suashfs"])
        except FileNotFoundError as err:
            e = err.strerror
        assert e == "No such file or directory"

    def test_use_llvm(self, setup_method_fixture, new_dir):
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-use-llvm": True,
            },
        )
        assert plugin._llvm_version == "1"
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "debhelper",
            "fakeroot",
            "gcc",
            "cmake",
            "cryptsetup",
            "kmod",
            "kpartx",
            "lld",
            "llvm",
            "systemd",
        }
        plugin.get_build_environment()
        cmd = plugin.get_build_commands()
        targets = f"{plugin.kernel_image_target} modules"
        if plugin._kernel_arch in ("arm", "arm64", "riscv64"):
            targets += " dtbs"
        assert (
            f'make -j$(nproc) -C ${{KERNEL_SRC}} O=${{CRAFT_PART_BUILD}} LLVM="1" {targets}'
            in cmd
        )

    def test_use_llvm_specific_version(self, setup_method_fixture, new_dir):
        version = "-10"
        plugin = setup_method_fixture(
            new_dir,
            properties={
                "kernel-use-llvm": version,
            },
        )
        assert plugin._llvm_version == version
        assert plugin.get_build_packages() == {
            "bc",
            "binutils",
            "debhelper",
            "fakeroot",
            "gcc",
            "cmake",
            "cryptsetup",
            "kmod",
            "kpartx",
            f"lld{version}",
            f"llvm{version}",
            "systemd",
        }
        plugin.get_build_environment()
        cmd = plugin.get_build_commands()
        targets = f"{plugin.kernel_image_target} modules"
        if plugin._kernel_arch in ("arm", "arm64", "riscv64"):
            targets += " dtbs"
        assert (
            f'make -j$(nproc) -C ${{KERNEL_SRC}} O=${{CRAFT_PART_BUILD}} LLVM="{version}" {targets}'
            in cmd
        )

    def test_bad_llvm_version(self, setup_method_fixture, new_dir):
        with pytest.raises(ValueError):
            setup_method_fixture(
                new_dir,
                properties={
                    "kernel-use-llvm": "-badsuffix",
                },
            )


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

_DEB_TO_KERNEL_ARCH_TRANSLATIONS = {
    "arm64": "arm64",
    "armhf": "arm",
    "riscv64": "riscv",
    "amd64": "x86",
}

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

_clean_old_build_cmd = [
    textwrap.dedent(
        """
        echo "Cleaning previous build first..."
        [ -e ${CRAFT_PART_INSTALL}/modules ] && rm -rf ${CRAFT_PART_INSTALL}/modules
        [ -L ${CRAFT_PART_INSTALL}/lib/modules ] && rm -rf ${CRAFT_PART_INSTALL}/lib/modules
        """
    )
]

_prepare_config_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${CRAFT_PART_BUILD}/.config ]; then",
    "\tmake -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} defconfig",
    "fi",
]

_prepare_config_custom_cc_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${CRAFT_PART_BUILD}/.config ]; then",
    '\tmake -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} CC="my-gcc" defconfig',
    "fi",
]

_prepare_config_defconfig_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${CRAFT_PART_BUILD}/.config ]; then",
    "\tcp arch/arm64/configs/snappy_defconfig ${CRAFT_PART_BUILD}/.config",
    "fi",
]

_prepare_config_flavour_cmd = [
    'echo "Preparing config..."',
    "if [ ! -e ${CRAFT_PART_BUILD}/.config ]; then",
    textwrap.dedent(
        """	echo "Assembling Ubuntu config..."
	if [ -f ${KERNEL_SRC}/debian/rules ] && [ -x ${KERNEL_SRC}/debian/rules ]; then
		# Generate Ubuntu kernel configs
		pushd ${KERNEL_SRC}
		fakeroot debian/rules clean genconfigs || true
		popd

		# Pick the right kernel .config for the target arch and flavour
		ubuntuconfig=${KERNEL_SRC}/CONFIGS/${DEB_ARCH}-config.flavour.raspi
		cat ${ubuntuconfig} > ${CRAFT_PART_BUILD}/.config

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
    "echo 'CONFIG_DEBUG_INFO=n\nCONFIG_DM_CRYPT=y' > ${CRAFT_PART_BUILD}/.config_snap",
    "cat ${CRAFT_PART_BUILD}/.config >> ${CRAFT_PART_BUILD}/.config_snap",
    "echo 'CONFIG_DEBUG_INFO=n\nCONFIG_DM_CRYPT=y' >> ${CRAFT_PART_BUILD}/.config_snap",
    "mv ${CRAFT_PART_BUILD}/.config_snap ${CRAFT_PART_BUILD}/.config",
]

_remake_old_config_cmd = [
    'echo "Remaking oldconfig...."',
    "bash -c 'yes \"\" || true' | make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} oldconfig",
]

_remake_old_config_custom_cc_cmd = [
    'echo "Remaking oldconfig...."',
    "bash -c 'yes \"\" || true'"
    ' | make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} CC="my-gcc" oldconfig',
]

_remake_old_config_clang_cmd = [
    'echo "Remaking oldconfig...."',
    " ".join(
        [
            "bash -c 'yes \"\" || true' | make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD}",
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
            "bash -c 'yes \"\" || true' | make -j1 -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD}",
            "ARCH=arm CROSS_COMPILE=${CRAFT_ARCH_TRIPLET}-",
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
        "${CRAFT_PART_BUILD}/.config",
    ],
)

_build_kernel_cmd = [
    "make -j$(nproc) -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} Image.gz modules dtbs",
]

_build_kernel_x86_cmd = [
    "make -j$(nproc) -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} bzImage modules",
]

_build_kernel_custom_cc_cmd = [
    'make -j$(nproc) -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} CC="my-gcc" Image.gz modules dtbs',
]

_build_kernel_x86_custom_cc_cmd = [
    'make -j$(nproc) -C ${KERNEL_SRC} O=${CRAFT_PART_BUILD} CC="my-gcc" bzImage modules',
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

_build_kernel_clang_image_x86_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
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
            "Image.gz modules pi3.dtb pi3b.dtb pi4.dtb pi/cm3.dtb pi/cm4.dtb",
        ],
    ),
]

_build_kernel_dtbs_x86_cmd = [
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "bzImage modules pi3.dtb pi3b.dtb pi4.dtb pi/cm3.dtb pi/cm4.dtb",
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

_install_kernel_x86_cmd = [
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

_install_kernel_custom_cc_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            'CC="my-gcc"',
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
            "dtbs_install INSTALL_DTBS_PATH=${CRAFT_PART_INSTALL}/dtbs",
            "firmware_install INSTALL_FW_PATH=${CRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_install_kernel_x86_custom_cc_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            'CC="my-gcc"',
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
            "firmware_install INSTALL_FW_PATH=${CRAFT_PART_INSTALL}/lib/firmware",
        ],
    ),
]

_install_kernel_no_dtbs_no_firmware_cmd = [
    " ".join(
        [
            "make -j$(nproc) -C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            "CONFIG_PREFIX=${CRAFT_PART_INSTALL}",
            "modules_install INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=${CRAFT_PART_INSTALL}",
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

_install_kernel_no_firmware_clang_x86_cmd = [
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

_install_dtbs_cmd = [
    'echo "Copying custom dtbs..."',
    "mkdir -p ${CRAFT_PART_INSTALL}/dtbs",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi3.dtb ${CRAFT_PART_INSTALL}/dtbs/pi3.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi3b.dtb ${CRAFT_PART_INSTALL}/dtbs/pi3b.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi4.dtb ${CRAFT_PART_INSTALL}/dtbs/pi4.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi/cm3.dtb ${CRAFT_PART_INSTALL}/dtbs/cm3.dtb",
    "ln -f ${KERNEL_BUILD_ARCH_DIR}/dts/pi/cm4.dtb ${CRAFT_PART_INSTALL}/dtbs/cm4.dtb",
]

_install_config_cmd = [
    'echo "Installing kernel config..."',
    "ln -f ${CRAFT_PART_BUILD}/.config ${CRAFT_PART_INSTALL}/config-${KERNEL_RELEASE}",
]


_finalize_install_cmd = [
    textwrap.dedent(
        """

        echo "Finalizing install directory..."
        # upstream kernel installs under $INSTALL_MOD_PATH/lib/modules/
        # but snapd expects modules/ and firmware/
        mv ${CRAFT_PART_INSTALL}/lib/modules ${CRAFT_PART_INSTALL}/
        # remove symlinks modules/*/build and modules/*/source
        rm -f ${CRAFT_PART_INSTALL}/modules/*/build ${CRAFT_PART_INSTALL}/modules/*/source
        # if there is firmware dir, move it to snap root
        # this could have been from stage packages or from kernel build
        [ -d ${CRAFT_PART_INSTALL}/lib/firmware ] && mv ${CRAFT_PART_INSTALL}/lib/firmware ${CRAFT_PART_INSTALL}
        # create symlinks for modules and firmware for convenience
        ln -sf ../modules ${CRAFT_PART_INSTALL}/lib/modules
        ln -sf ../firmware ${CRAFT_PART_INSTALL}/lib/firmware
        """
    )
]


_clone_zfs_cmd = [
    textwrap.dedent(
        """
        if [ ! -d ${CRAFT_PART_BUILD}/zfs ]; then
        	echo "cloning zfs..."
        	git clone --depth=1 https://github.com/openzfs/zfs ${CRAFT_PART_BUILD}/zfs -b master
        fi
        """
    )
]

_build_zfs_cmd = [
    textwrap.dedent(
        """
        echo "Building zfs modules..."
        cd ${CRAFT_PART_BUILD}/zfs
        ./autogen.sh
        ./configure --with-linux=${KERNEL_SRC} --with-linux-obj=${CRAFT_PART_BUILD} \
--with-config=kernel --host=${CRAFT_ARCH_TRIPLET}
        make -j$(nproc)
        make install DESTDIR=${CRAFT_PART_INSTALL}/zfs
        release_version="$(ls ${CRAFT_PART_INSTALL}/modules)"
        mv ${CRAFT_PART_INSTALL}/zfs/lib/modules/${release_version}/extra \
${CRAFT_PART_INSTALL}/modules/${release_version}
        rm -rf ${CRAFT_PART_INSTALL}/zfs
        echo "Rebuilding module dependencies"
        depmod -b ${CRAFT_PART_INSTALL} ${release_version}
        """
    )
]

_build_perf_cmd = [
    'echo "Building perf binary..."',
    'mkdir -p "${CRAFT_PART_BUILD}/tools/perf"',
    " ".join(
        [
            "make -j$(nproc)",
            "-C ${KERNEL_SRC}",
            "O=${CRAFT_PART_BUILD}",
            '-C "${KERNEL_SRC}/tools/perf"',
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
            '-C "${KERNEL_SRC}/tools/perf"',
            'O="${CRAFT_PART_BUILD}/tools/perf"',
        ],
    ),
    'install -Dm0755 "${CRAFT_PART_BUILD}/tools/perf/perf" "${CRAFT_PART_INSTALL}/bin/perf"',
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
