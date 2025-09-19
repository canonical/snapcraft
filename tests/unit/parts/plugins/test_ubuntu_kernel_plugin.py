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
import dataclasses
import functools
import os
import pathlib
import re
from unittest import mock

import pytest
from craft_parts import Part, PartInfo, ProjectInfo

from snapcraft import errors
from snapcraft.parts.plugins import UbuntuKernelPlugin, ubuntu_kernel_plugin

KERNEL_VERSION_MOCK_VALUE = ("5.15.0-143.153", "5.15.0-143")


def build_from_debpkg_cmds() -> list[str]:
    """Return build commands for building from deb packages.

    Note: All snapcraft root paths are normalised to for comparison `;parts`

    Returns:
        List of build commands for building from binary deb packages.
    """
    return [
        "rsync -aH ;parts/ubuntu-kernel/src/*.deb ;parts/ubuntu-kernel/build/kernel",
    ]


@pytest.fixture
def build_cmds_environ() -> None:
    """Set the common build commands environment."""
    os.environ |= {
        "SNAP_CONTEXT": "snap-context",
        "SNAP": "snap",
        "SNAP_VERSION": "1.0.0",
    }


def normalise_actual_cmds(raw_cmds: list[str]) -> list[str]:
    """Normalise the actual commands for comparison.

    Removes empty lines, comments, and normalises paths to a common ';' root
    character.

    Args:
        raw_cmds: List of raw command strings.

    Returns:
        List of normalised command strings.
    """
    actual_cmds = [x.strip() for y in raw_cmds for x in y.split("\n")]
    actual_cmds = [re.sub(r"[^ ]*/parts", ";parts", x) for x in actual_cmds if x]
    actual_cmds = [re.sub(r"^\s*#.*$", "", x) for x in actual_cmds]
    actual_cmds = [x for x in actual_cmds if x]
    return actual_cmds


def get_cross_build_cmds(target_arch: str | None) -> tuple[list[str], list[str]]:
    """Get the cross-build commands snippets.

    Args:
        target_arch: The target architecture for the build.
    Returns:
        Tuple of pre-debian environment and post-debian environment commands
        for cross-compilation.
    """
    cross_build_cmds_pre_debenv = (
        []
        if target_arch == "amd64"
        else [
            "export ARCH=arm64",
            "export CROSS_COMPILE=aarch64-linux-gnu-",
        ]
    )
    cross_build_cmds_post_debenv = (
        []
        if target_arch == "amd64"
        else [
            'export "$(dpkg-architecture -aarm64)"',
        ]
    )
    return cross_build_cmds_pre_debenv, cross_build_cmds_post_debenv


def get_kernel_image_target_cmds(
    target_arch: str | None, kernel_image_target: str | None
) -> list[str]:
    """Get the kernel image target command snippet.

    Args:
        target_arch: The target architecture for the build.
        kernel_image_target: The kernel image target file name.

    Returns:
        List of command snippets to update the kernel image target for debian
        build rules.
    """
    kernel_image_target_cmds = []
    if kernel_image_target:
        kernel_image_target_cmds = [
            'echo "Updating build target image"',
            "sed -i \\",
            f"'s/^\\s*build_image.*/build_image = {kernel_image_target}/g' \\",
            f"debian.master/rules.d/{target_arch}.mk",
            "sed -i \\",
            f"'s|^\\s*kernel_file.*|kernel_file = arch/{target_arch}/boot/{kernel_image_target}|g' \\",
            f"debian.master/rules.d/{target_arch}.mk",
        ]
    return kernel_image_target_cmds


def get_kernel_tool_cmds(
    target_arch: str | None, kernel_tools: list[str] | None
) -> list[str]:
    """Get the kernel tools command snippets.

    Args:
        target_arch: The target architecture for the build.
        kernel_tools: List of kernel tools to enable in the build.
    Returns:
        List of command snippets to update the kernel tools selection in debian
        build rules.
    """
    kernel_tools_cmds = []
    if kernel_tools:
        kernel_tools_cmds = [
            'echo "Updating kernel tools selection"',
            r"sed -i 's/^\\s*do_tools_\\(.*\\)\\s*=.*/do_tools_\\1 = false/g' " + "\\",
            f"debian.master/rules.d/{target_arch}.mk",
        ]
        for tool in kernel_tools:
            kernel_tools_cmds += [
                f"sed -i 's/^\\s*do_tools_{tool}\\s*=.*/do_tools_{tool} = true/g' \\",
                f"debian.master/rules.d/{target_arch}.mk",
            ]
    return kernel_tools_cmds


def get_kernel_config_fragment_cmds(
    target_arch: str | None, kernel_config_fragments: list[str] | None
) -> list[str]:
    """Get the kernel config fragment command snippets.

    Args:
        target_arch: The target architecture for the build.
        kernel_config_fragments: List of kernel config fragments to apply.

    Returns:
        List of command snippets to update the kernel config fragments in the
        Ubuntu kernel annotations source.
    """
    kernel_config_fragment_cmds = []
    if kernel_config_fragments:
        kernel_config_fragment_cmds.append("{")
        for config in kernel_config_fragments:
            kernel_config_fragment_cmds.append(f'echo "{config}"')
        kernel_config_fragment_cmds.append("} > ;custom_config_fragment")
        kernel_config_fragment_cmds += [
            "./debian/scripts/misc/annotations \\",
            f"--arch {target_arch} \\",
            "--flavour generic \\",
            "--update ;custom_config_fragment",
        ]
    return kernel_config_fragment_cmds


def get_kernel_defconfig_cmds(
    target_arch: str | None, kernel_defconfig: str | None
) -> list[str]:
    """Get the kernel defconfig command snippets.

    Args:
        target_arch: The target architecture for the build.
        kernel_defconfig: The path to the kernel defconfig file.

    Returns:
        List of command snippets to update the kernel annotations from a
        defconfig file.
    """
    kernel_defconfig_cmds = []
    if kernel_defconfig:
        kernel_defconfig_cmds = [
            "./debian/scripts/misc/annotations \\",
            f"--arch {target_arch} \\",
            "--flavour generic \\",
            f"--import ;{kernel_defconfig}",
        ]
    return kernel_defconfig_cmds


def get_kernel_dkms_cmds(
    target_arch: str | None, kernel_dkms_modules: list[str] | None
) -> list[str]:
    """Get the kernel DKMS command snippets.

    Args:
        target_arch: The target architecture for the build.
        kernel_dkms_modules: List of DKMS modules to include in the build.
    Returns:
        List of command snippets to update the kernel DKMS modules debian build
        rules.
    """
    kernel_dkms_cmds = []
    if kernel_dkms_modules:
        for dkms in kernel_dkms_modules:
            kernel_dkms_cmds.append(f"apt show {dkms} > pkginfo")
            kernel_dkms_cmds.append(
                "source=$(grep \"Source:\" pkginfo | sed 's/Source: \\(.*\\)$/\\1/g')"
            )
            kernel_dkms_cmds.append(
                "version=$(grep \"Version:\" pkginfo | sed 's/Version: \\(.*\\)$/\\1/g')"
            )
            kernel_dkms_cmds.append(
                "repo=$(grep \"Section:\" pkginfo | sed 's/Section: \\(.*\\)\\/.*$/\\1/g')"
            )
            kernel_dkms_cmds.append('echo "${source} ${version} " \\')
            toks = dkms.split("-")
            kernel_dkms_cmds.append('"modulename=${source} " \\')
            kernel_dkms_cmds.append(
                f'"debpath=pool/${{repo}}/{dkms[0]}/%package%/{dkms}_%version%_all.deb arch={target_arch} " \\'
            )
            kernel_dkms_cmds.append(f'"rprovides={toks[0]}-modules " \\')
            kernel_dkms_cmds.append(
                f'"rprovides={dkms}" >> debian.master/dkms-versions'
            )
    return kernel_dkms_cmds


def build_from_source_cmds(
    target_arch: str,
    kernel_defconfig: str | None,
    kernel_config_fragments: list[str] | None,
    kernel_tools: list[str] | None,
    kernel_dkms_modules: list[str] | None,
    kernel_image_target: str | None,
) -> list[str]:
    """Return build commands for building from source.

    Note: All paths have been normalised to a common ';' root character.

    Args:
        target_arch: Optional target architecture for the build.
        kernel_defconfig: Optional path to the kernel defconfig file.
        kernel_config_fragments: Optional list of kernel config fragments to apply.
        kernel_tools: Optional list of kernel tools to enable in the build.
        kernel_dkms_modules: Optional list of DKMS modules to include in the build.
        kernel_image_target: Optional kernel image target file name.
    Returns:
        List of build commands for building from source.
    """
    cross_build_cmds_pre_debenv, cross_build_cmds_post_debenv = get_cross_build_cmds(
        target_arch
    )
    kernel_image_target_cmds = get_kernel_image_target_cmds(
        target_arch, kernel_image_target
    )
    kernel_tools_cmds = get_kernel_tool_cmds(target_arch, kernel_tools)
    kernel_config_fragment_cmds = get_kernel_config_fragment_cmds(
        target_arch, kernel_config_fragments
    )
    kernel_defconfig_cmds = get_kernel_defconfig_cmds(target_arch, kernel_defconfig)
    kernel_dkms_cmds = get_kernel_dkms_cmds(target_arch, kernel_dkms_modules)

    return (
        cross_build_cmds_pre_debenv
        + [
            "rsync -aH ;parts/ubuntu-kernel/src/ ;parts/ubuntu-kernel/build/kernel-src",
            "cd ;parts/ubuntu-kernel/build/kernel-src",
            ". debian/debian.env",
        ]
        + cross_build_cmds_post_debenv
        + kernel_tools_cmds
        + kernel_image_target_cmds
        + kernel_config_fragment_cmds
        + kernel_defconfig_cmds
        + kernel_dkms_cmds
        + [
            "fakeroot debian/rules clean",
            "fakeroot debian/rules updateconfigs || true",
            "fakeroot debian/rules printenv",
            "fakeroot debian/rules build-generic",
            "fakeroot debian/rules binary-generic",
            "fakeroot debian/rules binary-headers",
            "cd ;parts/ubuntu-kernel/build",
            "mv ;parts/ubuntu-kernel/build/*.deb ;parts/ubuntu-kernel/build/kernel",
        ]
    )


def build_cmds(
    build_from_binary_package: bool,
    target_arch: str,
    kernel_defconfig: str | None = None,
    kernel_config_fragments: list[str] | None = None,
    kernel_tools: list[str] | None = None,
    kernel_dkms_modules: list[str] | None = None,
    kernel_image_target: str | None = None,
) -> list[str]:
    """Return build commands for building the Ubuntu kernel from a binary
    debpkg or source.

    Note: All paths have been normalised to a common ';' root character.

    Args:
        build_from_binary_package: Whether to build from binary packages.
        target_arch: The target architecture for the build.
        kernel_defconfig: Optional path to the kernel defconfig file.
        kernel_config_fragments: Optional list of kernel config fragments to apply.
        kernel_tools: Optional list of kernel tools to enable in the build.
        kernel_dkms_modules: Optional list of DKMS modules to include in the build.
        kernel_image_target: Optional kernel image target file name.
    Returns:
        List of common build commands for building the Ubuntu kernel.
    """
    build_type_cmds = (
        build_from_debpkg_cmds()
        if build_from_binary_package
        else build_from_source_cmds(
            target_arch=target_arch,
            kernel_defconfig=kernel_defconfig,
            kernel_config_fragments=kernel_config_fragments,
            kernel_tools=kernel_tools,
            kernel_dkms_modules=kernel_dkms_modules,
            kernel_image_target=kernel_image_target,
        )
    )

    dpkg_deb_linux_image_cmd = (
        [
            f"dpkg-deb -R linux-image-5.15.0-143-generic_5.15.0-143.153_{target_arch}.deb unpacked-linux-image",
        ]
        if build_from_binary_package
        else [
            f"dpkg-deb -R linux-image-unsigned-5.15.0-143-generic_5.15.0-143.153_{target_arch}.deb unpacked-linux-image",
        ]
    )
    cmds = (
        [
            "env",
            "cd ;parts/ubuntu-kernel/build",
            "rm -fr ;parts/ubuntu-kernel/build/kernel/",
            "rm -fr ;parts/ubuntu-kernel/build/unpacked-*",
            "rm -fr ;parts/ubuntu-kernel/install/*",
            "mkdir ;parts/ubuntu-kernel/build/kernel",
        ]
        + build_type_cmds
        + [
            "cd ;parts/ubuntu-kernel/build/kernel",
        ]
        + dpkg_deb_linux_image_cmd
        + [
            f"dpkg-deb -x linux-modules-5.15.0-143-generic_5.15.0-143.153_{target_arch}.deb unpacked-linux-modules",
            f"dpkg-deb -x linux-modules-extra-5.15.0-143-generic_5.15.0-143.153_{target_arch}.deb unpacked-linux-modules",
            "mv unpacked-linux-image/* ;parts/ubuntu-kernel/install",
            "mkdir ;parts/ubuntu-kernel/install/lib",
            "mv unpacked-linux-modules/lib/modules ;parts/ubuntu-kernel/install/lib/",
            "mv unpacked-linux-modules/boot/* ;parts/ubuntu-kernel/install/boot/",
            "depmod -b ;parts/ubuntu-kernel/install 5.15.0-143-generic",
            "mv ;parts/ubuntu-kernel/install/boot/* ;parts/ubuntu-kernel/install/",
            "ln -f ;parts/ubuntu-kernel/install/vmlinuz-5.15.0-143-generic ;parts/ubuntu-kernel/install/kernel.img",
            "mv ;parts/ubuntu-kernel/install/lib/modules ;parts/ubuntu-kernel/install/",
            "DTBS=unpacked-linux-firmware/lib/firmware/5.15.0-143/device-tree",
            '[ -d "${DTBS}" ] && mv "${DTBS}" ;parts/ubuntu-kernel/install/dtbs',
            "FIRMWARE=unpacked-linux-firmware/lib/firmware/5.15.0-143",
            '[ -d "${FIRMWARE}" ] && mv "${FIRMWARE}" ;parts/ubuntu-kernel/install/firmware',
            "rm -rf ;parts/ubuntu-kernel/install/boot",
        ]
    )
    cmd_list = [x.strip() for x in cmds if x]
    return cmd_list


@dataclasses.dataclass
class BuildParameters:
    """Parameters for the build normally provided by snapcraft."""

    def __init__(
        self,
        base: str,
        arch_build_on: str,
        arch_build_for: str,
        arch_triplet_build_for: str,
    ):
        self.base = base
        self.arch_build_for = arch_build_for
        self.arch_build_on = arch_build_on
        self.arch_triplet_build_for = arch_triplet_build_for


@functools.lru_cache
def get_project_info_parameters() -> list[BuildParameters]:
    """Generate a ProjectInfo object common to all tests."""
    return [
        BuildParameters(
            base=base,
            arch_build_on="amd64",
            arch_build_for=arch_build_for,
            arch_triplet_build_for=arch_triplet_build_for,
        )
        for base in ["core22", "core24"]
        for arch_build_for in ["amd64", "arm64"]
        for arch_triplet_build_for in [
            "x86_64-linux-gnu" if arch_build_for == "amd64" else "aarch64-linux-gnu"
        ]
    ]


def get_test_fixture_ids() -> list[str]:
    """Generate fixture ids to help identify tests in logs."""
    params = get_project_info_parameters()
    return [f"{x.base}, {x.arch_build_for}" for x in params]


@pytest.fixture
def setup_method_fixture():
    """Fixture to set up common UbuntuKernelPlugin elements for testing."""

    def _setup_method_fixture(
        build_params: BuildParameters,
        new_dir: pathlib.Path,
        properties: dict[str, str] | None = None,
    ) -> UbuntuKernelPlugin:
        if not properties:
            properties = {"ubuntu-kernel-release-name": "jammy"}

        part = Part("ubuntu-kernel", {})

        project_info = ProjectInfo(
            application_name="test",
            cache_dir=new_dir,
            arch=build_params.arch_build_for,
            base=build_params.base,
        )

        part_info = PartInfo(project_info=project_info, part=part)
        properties_class = UbuntuKernelPlugin.properties_class.unmarshal(properties)
        return UbuntuKernelPlugin(
            properties=properties_class,
            part_info=part_info,
        )

    yield _setup_method_fixture


class TestPluginUbuntuKenrel:
    """UbuntuKernel plugin tests."""

    def test_property_requires_source_or_release_name(self, new_dir):
        """Test the property validates source and release name."""
        properties = {
            "plugin-name": "ubuntu-kernel",
        }
        with pytest.raises(errors.SnapcraftError):
            UbuntuKernelPlugin.properties_class.unmarshal(properties)

        properties["ubuntu-kernel-release-name"] = "hello"
        # Should not raise
        _ = UbuntuKernelPlugin.properties_class.unmarshal(properties)

        properties["source"] = "git://git-repo.git"
        with pytest.raises(errors.SnapcraftError):
            UbuntuKernelPlugin.properties_class.unmarshal(properties)

    @pytest.mark.parametrize(
        "invalid_property, invalid_property_value",
        [
            ("ubuntu-kernel-defconfig", "my-defconfig-file"),
            ("ubuntu-kernel-config", ["CONFIG_FOO=1", "CONFIG_BAR=2"]),
            ("ubuntu-kernel-tools", ["perf", "cpupower", "bpftool"]),
            ("ubuntu-kernel-dkms", ["vpoll-dkms", "r8125-dkms"]),
        ],
    )
    def test_property_validation_with_binary_package(
        self,
        invalid_property,
        invalid_property_value,
        new_dir,
    ):
        """Test validation of binary package and config."""
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-use-binary-package": True,
            invalid_property: invalid_property_value,
        }
        with pytest.raises(errors.SnapcraftError):
            UbuntuKernelPlugin.properties_class.unmarshal(properties)

        properties["ubuntu-kernel-use-binary-package"] = False
        # Should not raise
        _ = UbuntuKernelPlugin.properties_class.unmarshal(properties)

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_snaps(self, build_params, new_dir, setup_method_fixture):
        """Test the expected build packages for building the Ubuntu kernel."""
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
        )
        assert plugin.get_build_snaps() == set()

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_packages(self, build_params, new_dir, setup_method_fixture):
        """Test the expected build packages for building the Ubuntu kernel."""
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
        )
        expected_common_packages = {
            "debhelper-compat",
            "cpio",
            "kmod",
            "makedumpfile",
            "libcap-dev",
            "libelf-dev",
            "libnewt-dev",
            "libiberty-dev",
            "default-jdk-headless",
            "java-common",
            "rsync",
            "libdw-dev",
            "libpci-dev",
            "pkg-config",
            "python3-dev",
            "flex",
            "bison",
            "libunwind8-dev",
            "liblzma-dev",
            "openssl",
            "libssl-dev",
            "libaudit-dev",
            "bc",
            "gawk",
            "libudev-dev",
            "autoconf",
            "automake",
            "libtool",
            "uuid-dev",
            "libnuma-dev",
            "dkms",
            "curl",
            "zstd",
            "pahole",
            "bzip2",
            "debhelper",
            "fakeroot",
            "lz4",
            "python3",
            "dwarfdump",
            "git",
        }
        additional_cross_compile_packages = {
            f"binutils-{plugin.part_info.arch_triplet_build_for}",
            f"gcc-{plugin.part_info.arch_triplet_build_for}",
            f"libc6-dev-{plugin.part_info.target_arch}-cross",
        }
        expected_packages = {"core22": {}, "core24": {}}
        expected_packages["core22"]["amd64"] = expected_common_packages
        expected_packages["core22"]["arm64"] = (
            expected_common_packages | additional_cross_compile_packages
        )

        expected_packages["core24"]["amd64"] = expected_common_packages | {
            "python3-setuptools",
            "libtraceevent-dev",
            "libtracefs-dev",
            "clang-18",
            "rustc",
            "rust-src",
            "rustfmt",
            "bindgen-0.65",
            "libstdc++-13-dev",
        }
        expected_packages["core24"]["arm64"] = (
            expected_packages["core24"]["amd64"]
            | additional_cross_compile_packages
            | {
                f"libstdc++-13-dev-{plugin.part_info.target_arch}-cross",
            }
        )
        assert (
            expected_packages[build_params.base][build_params.arch_build_for]
            == plugin.get_build_packages()
        )

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_environment(self, build_params, new_dir, setup_method_fixture):
        """Test the expected build packages for building the Ubuntu kernel."""
        plugin = setup_method_fixture(new_dir=new_dir, build_params=build_params)
        common_build_env = {}
        cross_compile_build_env = {
            "ARCH": build_params.arch_build_for,
            "CROSS_COMPILE": build_params.arch_triplet_build_for,
            "DEB_HOST_ARCH": build_params.arch_build_for,
            "DEB_BUILD_ARCH": build_params.arch_build_on,
        }
        expected_build_env = {
            "core22": {
                "amd64": common_build_env.copy(),
                "arm64": {**common_build_env, **cross_compile_build_env},
            },
            "core24": {
                "amd64": common_build_env.copy(),
                "arm64": {**common_build_env, **cross_compile_build_env},
            },
        }
        assert (
            expected_build_env[build_params.base][build_params.arch_build_for]
            == plugin.get_build_environment()
        )

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_pull_commands_with_source_url(
        self, build_params, new_dir, setup_method_fixture
    ):
        """Test the expected pull commands when a source is provided."""
        properties = {
            "plugin-name": "ubuntu-kernel",
            "source": "git://git-repo.git",
            "source-type": "git",
            "source-depth": 1,
            "source-branch": "master-next",
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )
        result = plugin.get_pull_commands()
        assert result == []

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_pull_commands_with_release_name_build_from_source(
        self, build_params, new_dir, setup_method_fixture
    ):
        """Test the expected pull commands when release-name provided."""
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )
        result = plugin.get_pull_commands()
        expected_clone_cmd = (
            "git clone --depth=1 --branch=master-next "
            "https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/jammy "
            "."
        )
        assert expected_clone_cmd in result[0]

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_pull_commands_with_release_name_from_debpkg_binary(
        self, build_params, new_dir, setup_method_fixture
    ):
        """Test the expected pull commands when release-name provided."""
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-use-binary-package": True,
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )
        expected_cmds = [
            r"apt-get update",
            f"KERNEL_ABI=$(apt show linux-image-generic:{build_params.arch_build_for}/jammy |",
            "    grep Version |",
            r"    sed 's/Version: \(.*\)$/\1/g' |",
            "    cut -d. -f1-4 |",
            r"    sed 's/\([0-9]\+\.[0-9]\+\.[0-9]\+\).\([0-9]\+\).*$/\1-\2/g'",
            'echo "Kernel ABI: ${KERNEL_ABI}',
            f'apt download linux-image-"${{KERNEL_ABI}}"-generic:{build_params.arch_build_for}',
            f'apt download linux-modules-"${{KERNEL_ABI}}"-generic:{build_params.arch_build_for}',
            f'apt download linux-modules-extra-"${{KERNEL_ABI}}"-generic:{build_params.arch_build_for}',
            "apt download linux-firmware",
        ]
        result = plugin.get_pull_commands()
        assert all(line in result[0] for line in expected_cmds)

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_use_binary_package(
        self,
        build_params,
        new_dir,
        setup_method_fixture,
        build_cmds_environ,
    ):
        """test the expected pull commands when release-name provided."""
        ubuntu_kernel_plugin.kernel_version_from_debpkg_file = mock.MagicMock(
            return_value=KERNEL_VERSION_MOCK_VALUE
        )
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-use-binary-package": True,
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )

        common_cmds = build_cmds(
            build_from_binary_package=True, target_arch=build_params.arch_build_for
        )
        actual_cmds = normalise_actual_cmds(plugin.get_build_commands())
        assert actual_cmds == common_cmds

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_source_build_stock_kernel(
        self,
        build_params,
        new_dir,
        setup_method_fixture,
        build_cmds_environ,
    ):
        """test the expected pull commands when release-name provided."""

        ubuntu_kernel_plugin.kernel_version_from_source_tree = mock.MagicMock(
            return_value=KERNEL_VERSION_MOCK_VALUE
        )
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )

        common_cmds = build_cmds(
            build_from_binary_package=False, target_arch=build_params.arch_build_for
        )
        actual_cmds = normalise_actual_cmds(plugin.get_build_commands())
        assert actual_cmds == common_cmds

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_source_build_with_extra_kernel_tools(
        self,
        build_params,
        new_dir,
        setup_method_fixture,
        build_cmds_environ,
    ):
        """test the expected pull commands when release-name provided."""

        ubuntu_kernel_plugin.kernel_version_from_source_tree = mock.MagicMock(
            return_value=KERNEL_VERSION_MOCK_VALUE
        )
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-tools": ["cpupower", "perf", "bpftool"],
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )

        common_cmds = build_cmds(
            build_from_binary_package=False,
            target_arch=build_params.arch_build_for,
            kernel_tools=properties["ubuntu-kernel-tools"],
        )

        actual_cmds = normalise_actual_cmds(plugin.get_build_commands())
        assert actual_cmds == common_cmds

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_source_build_with_defconfig(
        self,
        build_params,
        new_dir,
        setup_method_fixture,
        build_cmds_environ,
    ):
        """test the expected pull commands when release-name provided."""

        ubuntu_kernel_plugin.kernel_version_from_source_tree = mock.MagicMock(
            return_value=KERNEL_VERSION_MOCK_VALUE
        )
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-defconfig": "my-defconfig-file",
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )

        common_cmds = build_cmds(
            build_from_binary_package=False,
            target_arch=build_params.arch_build_for,
            kernel_defconfig=properties["ubuntu-kernel-defconfig"],
        )

        actual_cmds = normalise_actual_cmds(plugin.get_build_commands())
        actual_cmds = [
            re.sub(r"[^ ]*/my-defconfig-file", ";my-defconfig-file", x)
            for x in actual_cmds
            if x
        ]
        assert actual_cmds == common_cmds

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_source_build_with_config_fragments(
        self,
        build_params,
        new_dir,
        setup_method_fixture,
        build_cmds_environ,
    ):
        """test the expected pull commands when release-name provided."""

        ubuntu_kernel_plugin.kernel_version_from_source_tree = mock.MagicMock(
            return_value=KERNEL_VERSION_MOCK_VALUE
        )
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-config": ["CONFIG_FOO=y", "CONFIG_BAR=m"],
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )

        common_cmds = build_cmds(
            build_from_binary_package=False,
            target_arch=build_params.arch_build_for,
            kernel_config_fragments=properties["ubuntu-kernel-config"],
        )

        actual_cmds = normalise_actual_cmds(plugin.get_build_commands())
        # Ignore comments written to the config fragment file
        actual_cmds = [re.sub(r'^\s*echo "#.*$', "", x) for x in actual_cmds if x]
        actual_cmds = [re.sub(r'^\s*echo ""', "", x) for x in actual_cmds if x]
        # Normalise the path for comparison
        actual_cmds = [
            re.sub(r"[^ ]*/custom_config_fragment", ";custom_config_fragment", x)
            for x in actual_cmds
            if x
        ]
        assert actual_cmds == common_cmds

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_source_build_with_image_target(
        self,
        build_params,
        new_dir,
        setup_method_fixture,
        build_cmds_environ,
    ):
        """test the expected pull commands when release-name provided."""
        ubuntu_kernel_plugin.kernel_version_from_source_tree = mock.MagicMock(
            return_value=KERNEL_VERSION_MOCK_VALUE
        )
        image_target = "vmlinux.strip"
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-image-target": image_target,
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )

        common_cmds = build_cmds(
            build_from_binary_package=False,
            target_arch=build_params.arch_build_for,
            kernel_image_target=image_target,
        )
        actual_cmds = normalise_actual_cmds(plugin.get_build_commands())
        assert actual_cmds == common_cmds

    @pytest.mark.parametrize(
        "build_params",
        get_project_info_parameters(),
        ids=get_test_fixture_ids(),
    )
    def test_get_build_commands_source_build_with_dkms_list(
        self,
        build_params,
        new_dir,
        setup_method_fixture,
        build_cmds_environ,
    ):
        """test the expected pull commands when release-name provided."""
        ubuntu_kernel_plugin.kernel_version_from_source_tree = mock.MagicMock(
            return_value=KERNEL_VERSION_MOCK_VALUE
        )
        properties = {
            "plugin-name": "ubuntu-kernel",
            "ubuntu-kernel-release-name": "jammy",
            "ubuntu-kernel-dkms": ["vpoll-dkms", "evdi-dkms", "r8125-dkms"],
        }
        plugin = setup_method_fixture(
            new_dir=new_dir,
            build_params=build_params,
            properties=properties,
        )

        common_cmds = build_cmds(
            build_from_binary_package=False,
            target_arch=build_params.arch_build_for,
            kernel_dkms_modules=properties["ubuntu-kernel-dkms"],
        )
        actual_cmds = normalise_actual_cmds(plugin.get_build_commands())
        assert actual_cmds == common_cmds
