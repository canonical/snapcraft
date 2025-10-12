# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
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

"""The Ubuntu kernel plugin for building Ubuntu Core kernel snaps."""

import os
import pathlib
import re
from typing import Literal, cast

import jinja2
import pydantic
from craft_cli import emit
from craft_parts import infos, plugins
from typing_extensions import Self, override

from snapcraft import errors

KERNEL_REPO_STEM = "https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux/+git/"
DEFAULT_RELEASE_NAME = {"core22": "jammy", "core24": "noble"}

_DEFAULT_KERNEL_IMAGE_TARGET = {
    "amd64": "bzImage",
    "i386": "bzImage",
    "armhf": "zImage",
    "arm64": "Image.gz",
    "powerpc": "uImage",
    "ppc64el": "vmlinux.strip",
    "s390x": "bzImage",
    "riscv64": "Image",
}

## These are set in the debian.masters/rules.d/<arch>.mk

_AVAILABLE_TOOLS = [
    "cpupower",
    "perf",
    "bpftool",
]


def kernel_abi_from_version(kernel_version: str) -> str:
    """Given the kernel version string extract the ABI component.

    Args:
        kernel_version (str): The kernel version string with ABI and spin
        number, e.g. "5.15.0-1012.13".
    Returns:
        str: The kernel version with ABI but no spin number, e.g. "5.15.0-1012".
    """
    rem = re.match(r"(\d+\.\d+\.\d+-\d+)\.\d+", kernel_version)
    if not rem:
        raise errors.SnapcraftError("cannot parse kernel version from changelog")
    return rem.group(1)


def kernel_version_from_source_tree(source_root: pathlib.Path) -> tuple[str, str]:
    """Given a changelog file path open it and extract the kernel version.

    Args:
        source_root: The path to the source root directory
    Returns:
        A tuple containing the full kernel version and kernel ABI version
    """
    changelog_file = source_root / "debian.master" / "changelog"
    with changelog_file.open("r") as fptr:
        version_line = fptr.readline()
    kernel_version = version_line.split("(")[1].split(")")[0]
    kernel_abi = kernel_abi_from_version(kernel_version)
    return kernel_version, kernel_abi


def kernel_version_from_debpkg_file(root_dir: pathlib.Path) -> tuple[str, str]:
    """Get the kernel version from debian package file names.

    Args:
        root_dir: The path to the directory containing the *.deb files.

    Returns:
        A tuple containing the full kernel version and kernel ABI version.
    """
    version_re = re.compile(r".*(\d+\.\d+\.\d+-\d+\.\d+).*\.deb")
    for filename in [
        pobj.name for pobj in sorted(root_dir.iterdir()) if pobj.is_file()
    ]:
        rem = version_re.search(filename)
        if rem:
            kernel_version = rem.group(1)
            kernel_abi = kernel_abi_from_version(kernel_version)
            return kernel_version, kernel_abi
    raise errors.SnapcraftError("cannot identify kernel version from Debian packages")


def kernel_launchpad_repository(release_name: str) -> str:
    """Get the kernel launchpad repository.

    Args:
        release_name: The name of the Ubuntu release, e.g. "jammy", "noble".
    Returns:
        The URL of the kernel source repository for the given release.
    """
    return KERNEL_REPO_STEM + release_name


class UbuntuKernelPluginProperties(plugins.properties.PluginProperties, frozen=True):
    """The part properties used by the Ubuntu kernel plugin."""

    plugin: Literal["ubuntu-kernel"] = "ubuntu-kernel"
    """Plugin name."""
    ubuntu_kernel_flavour: str = "generic"
    """The ubuntu kernel flavour, will be ignored if defconfig provided."""
    ubuntu_kernel_dkms: list[str] = []
    """Additional dkms."""
    ubuntu_kernel_release_name: str | None = None
    """Ubuntu release to build. Mutually exclusive with 'source'."""
    ubuntu_kernel_defconfig: str | None = None
    """Path to custom defconfig, relative to the project directory."""
    ubuntu_kernel_config: list[str] = []
    """Custom set of kernel configuration parameters."""
    ubuntu_kernel_image_target: str | None = None
    """Kernel image target type."""
    ubuntu_kernel_tools: list[str] = []
    """Kernel tools to include, e.g. perf."""
    ubuntu_kernel_use_binary_package: bool = False
    """Flag to use prebuilt kernel packages. Only valid with ubuntu-kernel-release-name."""

    @pydantic.model_validator(mode="after")
    def validate_release_name_and_source_exclusive(self) -> Self:
        """Enforce release_name and source options are mutually exclusive."""
        if self.ubuntu_kernel_release_name and self.source:
            raise errors.SnapcraftError(
                "cannot use 'ubuntu-kernel-release-name' and 'source' keys at same time"
            )
        if not self.ubuntu_kernel_release_name and not self.source:
            raise errors.SnapcraftError(
                "missing either 'ubuntu-kernel-release-name' or 'source' key"
            )
        return self

    @pydantic.model_validator(mode="after")
    def validate_binary_package_and_source_build_options_mutually_exclusive(
        self,
    ) -> Self:
        """Enforce binary package and source-only options are exclusive."""
        if self.ubuntu_kernel_use_binary_package:
            conflicting_options = [
                "source",
                "ubuntu_kernel_config",
                "ubuntu_kernel_defconfig",
                "ubuntu_kernel_image_target",
                "ubuntu_kernel_tools",
                "ubuntu_kernel_dkms",
            ]
            for option in conflicting_options:
                if getattr(self, option):
                    raise errors.SnapcraftError(
                        "'ubuntu-kernel-use-binary-package' and "
                        f"'{option.replace('_', '-')}' keys are mutually exclusive"
                    )
        return self

    @pydantic.field_validator("ubuntu_kernel_tools")
    @classmethod
    def validate_tool_list(cls, value: list[str]) -> list[str]:
        """Check the list of tools is in the available list."""
        unknown_tools = [tool for tool in value if tool not in _AVAILABLE_TOOLS]
        if unknown_tools:
            raise errors.SnapcraftError(
                "The following requested tools are not supported: "
                f"{unknown_tools!r}. Supported tools: {_AVAILABLE_TOOLS!r}"
            )
        return value


class UbuntuKernelPlugin(plugins.Plugin):
    """Plugin for the Ubuntu kernel snap build."""

    properties_class = UbuntuKernelPluginProperties

    def __init__(
        self, *, properties: plugins.PluginProperties, part_info: infos.PartInfo
    ) -> None:
        super().__init__(properties=properties, part_info=part_info)
        self.options = cast(UbuntuKernelPluginProperties, self._options)
        if part_info.base not in ("core22", "core24"):
            raise errors.SnapcraftError("only core22 and core24 bases are supported")
        self.release_name = self.options.ubuntu_kernel_release_name
        self.image_target = (
            self.options.ubuntu_kernel_image_target
            if self.options.ubuntu_kernel_image_target is not None
            else _DEFAULT_KERNEL_IMAGE_TARGET[part_info.arch_build_for]
        )

    @override
    def get_build_snaps(self) -> set[str]:
        return set()

    @override
    def get_build_packages(self) -> set[str]:
        # This should instead extract the build dependency list from the Debian
        #  source package. See https://warthogs.atlassian.net/browse/KE-427
        build_packages = {
            "common": frozenset(
                {
                    "autoconf",
                    "automake",
                    "bc",
                    "bison",
                    "bzip2",
                    "cpio",
                    "curl",
                    "debhelper",
                    "debhelper-compat",
                    "default-jdk-headless",
                    "dkms",
                    "dwarfdump",
                    "fakeroot",
                    "flex",
                    "gawk",
                    "git",
                    "java-common",
                    "kmod",
                    "libaudit-dev",
                    "libcap-dev",
                    "libdw-dev",
                    "libelf-dev",
                    "libiberty-dev",
                    "liblzma-dev",
                    "libnewt-dev",
                    "libnuma-dev",
                    "libpci-dev",
                    "libssl-dev",
                    "libtool",
                    "libudev-dev",
                    "libunwind8-dev",
                    "lz4",
                    "makedumpfile",
                    "openssl",
                    "pahole",  # not in control file
                    "pkg-config",
                    "python3",
                    "python3-dev",
                    "rsync",
                    "uuid-dev",
                    "zstd",
                    *(
                        {
                            f"binutils-{self._part_info.arch_triplet_build_for}",
                            f"gcc-{self._part_info.arch_triplet_build_for}",
                            f"libc6-dev-{self._part_info.target_arch}-cross",
                        }
                        if self._part_info.is_cross_compiling
                        else set()
                    ),
                }
            ),
            "core22": frozenset(),
            "core24": frozenset(
                {
                    "bindgen-0.65",
                    "clang-18",
                    "libstdc++-13-dev",
                    "libtraceevent-dev",
                    "libtracefs-dev",
                    "python3-setuptools",
                    "rust-src",
                    "rustc",
                    "rustfmt",
                    *(
                        {
                            f"libstdc++-13-dev-{self._part_info.target_arch}-cross",
                        }
                        if self._part_info.is_cross_compiling
                        else set()
                    ),
                }
            ),
        }
        return set(build_packages["common"] | build_packages[self._part_info.base])

    @override
    def get_build_environment(self) -> dict[str, str]:
        """Returns additional build environment variables."""
        emit.debug("Getting build environment")
        return (
            {}
            if not self._part_info.is_cross_compiling
            else {
                "ARCH": self._part_info.arch_build_for,
                "CROSS_COMPILE": self._part_info.arch_triplet_build_for,
                "DEB_HOST_ARCH": self._part_info.arch_build_for,
                "DEB_BUILD_ARCH": self._part_info.arch_build_on,
            }
        )

    @override
    def get_pull_commands(self) -> list[str]:
        """Get the commands to pull the source code for the part.

        This will clone the kernel source explicitly if no `source` URL is
        provided. If building with binary deb packages, it will fetch the
        debian packages.

        See jinja2 templates in snapcraft/templates/kernel/ for details.
        """
        emit.debug("Getting pull commands")
        if self.options.source:
            return super().get_pull_commands()
        if not self.release_name:
            raise errors.SnapcraftError(
                "missing either 'ubuntu-kernel-release-name' or 'source' key"
            )
        template_file = "kernel/ubuntu_kernel_get_pull_commands.sh.j2"
        env = jinja2.Environment(
            loader=jinja2.PackageLoader("snapcraft", "templates"), autoescape=True
        )
        template = env.get_template(template_file)
        source_repo_url = kernel_launchpad_repository(self.release_name)
        script = template.render(
            {
                "ubuntu_kernel_use_binary_package": self.options.ubuntu_kernel_use_binary_package,
                "ubuntu_kernel_release_name": self.release_name,
                "is_cross_compiling": self._part_info.is_cross_compiling,
                "target_arch": self._part_info.target_arch,
                "ubuntu_kernel_flavour": self.options.ubuntu_kernel_flavour,
                "source_repo_url": source_repo_url,
            }
        )
        return [script]

    @override
    def get_build_commands(self) -> list[str]:
        """Get the commands to build the part.

        The build command script is defined in the jinja2 templates under
        snapcraft/templates/kernel/.
        """
        emit.debug("Getting build commands")
        # Get the kernel version from the source files.
        if self.options.ubuntu_kernel_use_binary_package:
            kernel_version, kernel_abi = kernel_version_from_debpkg_file(
                self._part_info.part_src_dir
            )
        else:
            kernel_version, kernel_abi = kernel_version_from_source_tree(
                self._part_info.part_src_dir
            )

        template_file = "kernel/ubuntu_kernel_get_build_commands.sh.j2"
        env = jinja2.Environment(
            loader=jinja2.PackageLoader("snapcraft", "templates"), autoescape=True
        )
        template = env.get_template(template_file)
        script = template.render(
            {
                "craft_arch_build_for": self._part_info.arch_build_for,
                "craft_arch_build_on": self._part_info.arch_build_on,
                "craft_arch_triplet_build_for": self._part_info.arch_triplet_build_for,
                "craft_arch_triplet_build_on": self._part_info.arch_triplet_build_on,
                "craft_part_build_dir": self._part_info.part_build_dir,
                "craft_part_install_dir": self._part_info.part_install_dir,
                "craft_part_src_dir": self._part_info.part_src_dir,
                "craft_project_dir": self._part_info.project_dir,
                "has_ubuntu_kernel_config_fragments": bool(
                    self.options.ubuntu_kernel_config
                ),
                "has_ubuntu_kernel_defconfig": bool(
                    self.options.ubuntu_kernel_defconfig
                ),
                "has_ubuntu_kernel_image_target": bool(
                    self.options.ubuntu_kernel_image_target
                ),
                "is_cross_compiling": self._part_info.is_cross_compiling,
                "kernel_abi": kernel_abi,
                "kernel_version": kernel_version,
                "pkgfile_version_all": f"{kernel_abi}_{kernel_version}_all",
                # The package version can get quite long so to keep jinja2
                # templates readable it is substituted with a variable.
                "pkgfile_version_flavour": (
                    f"{kernel_abi}-{self.options.ubuntu_kernel_flavour}_"
                    f"{kernel_version}_{self._part_info.target_arch}"
                ),
                "snap_context": os.environ["SNAP_CONTEXT"],
                "snap_data_path": os.environ["SNAP"],
                "snap_version": os.environ["SNAP_VERSION"],
                "target_arch": self._part_info.target_arch,
                "ubuntu_kernel_config": self.options.ubuntu_kernel_config,
                "ubuntu_kernel_defconfig": self.options.ubuntu_kernel_defconfig,
                "ubuntu_kernel_dkms": self.options.ubuntu_kernel_dkms,
                "ubuntu_kernel_flavour": self.options.ubuntu_kernel_flavour,
                "ubuntu_kernel_image_target": self.options.ubuntu_kernel_image_target,
                "ubuntu_kernel_tools": self.options.ubuntu_kernel_tools,
                "ubuntu_kernel_use_binary_package": self.options.ubuntu_kernel_use_binary_package,
            }
        )
        return [script]

    @override
    @classmethod
    def get_out_of_source_build(cls) -> bool:
        """Return whether the plugin performs out-of-source-tree builds."""
        return True
