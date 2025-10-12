# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
# pylint: disable=line-too-long,too-many-lines,attribute-defined-outside-init
#
# Copyright 2020-2025 Canonical Ltd.
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

import logging
import os
import pathlib
import re
from typing import Literal, cast

import jinja2
import pydantic
from craft_parts import infos, plugins
from overrides import overrides
from typing_extensions import Self

from snapcraft import errors

logger = logging.getLogger(__name__)

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
        raise errors.SnapcraftError("Failed to parse kernel version from changelog")
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
            return (kernel_version, kernel_abi)
    raise errors.SnapcraftError(
        "Failed to identify kernel version from deb package files."
    )


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
    """Ubuntu release to build. Mutually exclusive with `source`."""
    ubuntu_kernel_defconfig: str | None = None
    """Path to custom defconfig, relative to the project directory."""
    ubuntu_kernel_config: list[str] = []
    """Custom set of kernel configuration parameters."""
    ubuntu_kernel_image_target: str | None = None
    """Kernel image target type."""
    ubuntu_kernel_tools: list[str] = []
    """Kernel tools to include, e.g. perf."""
    ubuntu_kernel_use_binary_package: bool = False
    """Flag to use prebuilt kernel packages. Only valid with ubuntu-kerne-release-name."""

    @pydantic.model_validator(mode="after")
    def validate_release_name_and_source_exclusive(self) -> Self:
        """Enforce release_name and source options are mutually exclusive."""
        if self.ubuntu_kernel_release_name and self.source:
            raise errors.SnapcraftError(
                "`ubuntu-kernel-release-name` and `source` are mutually exclusive"
            )
        if not self.ubuntu_kernel_release_name and not self.source:
            raise errors.SnapcraftError(
                "must provide either `ubuntu-kernel-release-name` or `source`"
            )
        return self

    @pydantic.model_validator(mode="after")
    def validate_binary_package_and_source_build_options_mutually_exclusive(
        self,
    ) -> Self:
        """Enforce binary package and source-only options are exclusive."""
        if self.ubuntu_kernel_use_binary_package:
            blacklist_options = [
                "source",
                "ubuntu_kernel_config",
                "ubuntu_kernel_defconfig",
                "ubuntu_kernel_image_target",
                "ubuntu_kernel_tools",
                "ubuntu_kernel_dkms",
            ]
            for option in blacklist_options:
                if getattr(self, option):
                    raise errors.SnapcraftError(
                        f"`ubuntu-kernel-use-binary-package` and `{option}` are mutually exclusive"
                    )
        return self

    @pydantic.field_validator("ubuntu_kernel_tools")
    @classmethod
    def validate_tool_list(cls, value: list[str]) -> list[str]:
        """Check the list of tools is in the available list."""
        if any(x not in _AVAILABLE_TOOLS for x in value):
            raise errors.SnapcraftError(
                f"unknown tool provided, available tools: {_AVAILABLE_TOOLS}"
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
        self.part_info = part_info
        if part_info.base not in ("core22", "core24"):
            raise errors.SnapcraftError("only core22 and core24 bases are supported")
        self.release_name = self.options.ubuntu_kernel_release_name
        self.image_target = (
            self.options.ubuntu_kernel_image_target
            if self.options.ubuntu_kernel_image_target is not None
            else _DEFAULT_KERNEL_IMAGE_TARGET[part_info.arch_build_for]
        )

    @overrides
    def get_build_snaps(self) -> set[str]:
        return set()

    @overrides
    def get_build_packages(self) -> set[str]:
        # hardcoded for now
        build_packages = {
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
            "git",
            "libudev-dev",
            "autoconf",
            "automake",
            "libtool",
            "uuid-dev",
            "libnuma-dev",
            "dkms",
            "curl",
            "zstd",
            "pahole",  # not in control file
            "bzip2",
            "debhelper",
            "fakeroot",
            "lz4",
            "python3",
            "dwarfdump",
        }

        if self.part_info.base == "core24":
            build_packages |= {
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

        if self.part_info.is_cross_compiling:
            build_packages |= {
                f"binutils-{self.part_info.arch_triplet_build_for}",
                f"gcc-{self.part_info.arch_triplet_build_for}",
                f"libc6-dev-{self.part_info.target_arch}-cross",
            }
            if self.part_info.base == "core24":
                build_packages |= {
                    f"libstdc++-13-dev-{self.part_info.target_arch}-cross",
                }

        return build_packages

    @overrides
    def get_build_environment(self) -> dict[str, str]:
        """Returns additional build environment variables."""
        return (
            {}
            if not self.part_info.is_cross_compiling
            else {
                "ARCH": self.part_info.arch_build_for,
                "CROSS_COMPILE": self.part_info.arch_triplet_build_for,
                "DEB_HOST_ARCH": self.part_info.arch_build_for,
                "DEB_BUILD_ARCH": self.part_info.arch_build_on,
            }
        )

    @overrides
    def get_pull_commands(self) -> list[str]:
        """Get the commands to pull the source code for the part.

        This will clone the kernel source explicitly if no `source` URL is
        provided. If building with binary deb packages, it will fetch the
        debian packages.

        See jinja2 templates in snapcraft/templates/kernel/ for details.
        """
        if self.options.source:
            return super().get_pull_commands()
        if not self.release_name:
            raise errors.SnapcraftError(
                "must provide `ubuntu-kernel-release-name` or `source`"
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
                "is_cross_compiling": self.part_info.is_cross_compiling,
                "target_arch": self.part_info.target_arch,
                "ubuntu_kernel_flavour": self.options.ubuntu_kernel_flavour,
                "source_repo_url": source_repo_url,
            }
        )
        return [script]

    @overrides
    def get_build_commands(self) -> list[str]:
        """Get the commands to build the part.

        See jinja2 templates in snapcraft/templates/kernel/ for details.
        """
        logger.info("Setting build commands...")
        logger.info("*****************************")
        logger.info("self.options.source = %s", self.options.source)

        # Get the kernel version from the source files.
        if self.options.ubuntu_kernel_use_binary_package:
            kernel_version, kernel_abi = kernel_version_from_debpkg_file(
                self.part_info.part_src_dir
            )
        else:
            kernel_version, kernel_abi = kernel_version_from_source_tree(
                self.part_info.part_src_dir
            )

        template_file = "kernel/ubuntu_kernel_get_build_commands.sh.j2"
        env = jinja2.Environment(
            loader=jinja2.PackageLoader("snapcraft", "templates"), autoescape=True
        )
        template = env.get_template(template_file)
        script = template.render(
            {
                "craft_arch_build_for": self.part_info.arch_build_for,
                "craft_arch_build_on": self.part_info.arch_build_on,
                "craft_arch_triplet_build_for": self.part_info.arch_triplet_build_for,
                "craft_arch_triplet_build_on": self.part_info.arch_triplet_build_on,
                "craft_part_build_dir": self.part_info.part_build_dir,
                "craft_part_install_dir": self.part_info.part_install_dir,
                "craft_part_src_dir": self.part_info.part_src_dir,
                "craft_project_dir": self.part_info.project_dir,
                "has_ubuntu_kernel_config_fragments": bool(
                    self.options.ubuntu_kernel_config
                ),
                "has_ubuntu_kernel_defconfig": bool(
                    self.options.ubuntu_kernel_defconfig
                ),
                "has_ubuntu_kernel_image_target": bool(
                    self.options.ubuntu_kernel_image_target
                ),
                "is_cross_compiling": self.part_info.is_cross_compiling,
                "kernel_abi": kernel_abi,
                "kernel_version": kernel_version,
                "pkgfile_version_all": f"{kernel_abi}_{kernel_version}_all",
                # The package version can get quite long so to keep jinja2
                # templates readable it is substituted with a variable.
                "pkgfile_version_flavour": (
                    f"{kernel_abi}-{self.options.ubuntu_kernel_flavour}_"
                    f"{kernel_version}_{self.part_info.target_arch}"
                ),
                "snap_context": os.environ["SNAP_CONTEXT"],
                "snap_data_path": os.environ["SNAP"],
                "snap_version": os.environ["SNAP_VERSION"],
                "target_arch": self.part_info.target_arch,
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

    @classmethod
    def get_out_of_source_build(cls) -> bool:
        """Return whether the plugin performs out-of-source-tree builds."""
        return True
