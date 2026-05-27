# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2020-2022,2024 Canonical Ltd.
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

"""The kernel plugin for building kernel snaps.

The following kernel-specific options are provided by this plugin:

    - kernel-kdefconfig:
      (list of strings, default: defconfig))
      defconfig target to use as the base configuration. Multiple kdefconfigs
      may be specified. Their order matters (ones later in the list will take
      precedent over those earlier). These files should be in one of three places,
      depending on other options:
      If kernel-ubuntu-debian-package is true, they should be in
      ${CRAFT_PROJECT_DIR}/annotations and they should be in the annotations YAML
      format.
      If kernel-ubuntu-debian-package is false, they should be in
      either ${CRAFT_PART_SRC}/arch/${CRAFT_ARCH_BUILD_FOR}/configs or
      ${CRAFT_PART_SRC}/kernel/configs.

    - kernel-kconfigs:
      (list of strings; default: none)
      explicit list of configs to force; this will override the configs that
      were set as base through kernel-kdefconfig; dependent configs will be
      fixed using the defaults encoded in the kbuild config definitions. If you
      don't want default for one or more implicit configs coming out of these,
      just add them to this list as well.

    - kernel-tools
      (list of strings; default: none)
      a list of tools to build alongside the kernel. Accepted values are bpftool,
      cpupower, and perf

    - kernel-ubuntu-kconfigflavour:
      (string; default: generic)
      Ubuntu config flavour to use as base configuration. If provided this
      option wins over kernel-kdefconfig.

    - kernel-ubuntu-release-name
      (string; default: none)
      A specific Ubuntu release to fetch the source of and to build a kernel of.

    - kernel-ubuntu-abinumber
      (string; default: master-next)
      A string to specify either a particular kernel version and ABI, or a
      particular tag when cloning an Ubuntu kernel tree.

    - kernel-ubuntu-binary-package
      (boolean; default: False)
      Specifies whether or not a prebuilt debian kernel package from the archive should
      be used.

    - kernel-ubuntu-debian-package
      (boolean; default: False)
      Specifies whether to use the debian/rules file to build the kernel.

    - kernel-ubuntu-debian-dkms
      (list of strings; default: none)
      A list of DKMS package names to include in the kernel build. Only valid
      when kernel-ubuntu-debian-package is true. Each package must be available
      in the archive at build time.

This plugin supports cross compilation, for which the plugin expects the
build-environment is configured accordingly.
"""

from typing import Literal, cast

import pydantic
from craft_cli import emit
from craft_parts import errors, infos, plugins
from typing_extensions import Self, override

KERNEL_ARCH_FROM_SNAP_ARCH = {
    "i386": "x86",
    "amd64": "x86",
    "armhf": "arm",
    "arm64": "arm64",
    "ppc64el": "powerpc",
    "riscv64": "riscv",
    "s390x": "s390",
}

KernelTools = Literal["bpftool", "cpupower", "perf"]


class KernelPluginProperties(plugins.PluginProperties, frozen=True):
    """The part properties used by the Kernel plugin."""

    plugin: Literal["kernel"] = "kernel"

    kernel_kconfigs: list[str] = []
    kernel_kdefconfig: list[str] = ["defconfig"]
    kernel_tools: set[KernelTools] = pydantic.Field(default_factory=set)
    kernel_ubuntu_kconfigflavour: str = "generic"
    kernel_ubuntu_release_name: str = ""
    kernel_ubuntu_abinumber: str = "master-next"
    kernel_ubuntu_binary_package: bool = False
    kernel_ubuntu_debian_package: bool = False
    kernel_ubuntu_debian_dkms: list[str] = []

    @pydantic.model_validator(mode="after")
    def validate_release_name_and_source_exclusive(self) -> Self:
        """Enforce release_name and source options are mutually exclusive."""
        if self.kernel_ubuntu_release_name and self.source:
            raise errors.PartsError(
                brief="cannot use 'kernel-ubuntu-release-name' and 'source' at the same time",
                resolution="Remove either 'kernel-ubuntu-release-name' or 'source' from the part definition",
            )
        return self

    @pydantic.model_validator(mode="after")
    def validate_binary_package_and_source_build_options_mutually_exclusive(
        self,
    ) -> Self:
        """Enforce binary package and source-only options are exclusive."""
        kdefconfig = self.kernel_kdefconfig
        binary = self.kernel_ubuntu_binary_package

        if binary:
            conflicting_options = [
                "kernel_kconfigs",
                "kernel_kdefconfig",
                "kernel_ubuntu_release_name",
            ]
            for option in conflicting_options:
                if getattr(self, option):
                    if option == "kernel_kdefconfig" and kdefconfig == ["defconfig"]:
                        continue
                    emit.warning(
                        f"'{option}' will be ignored when 'kernel-ubuntu-binary-package' is set",
                    )
        return self

    @pydantic.model_validator(mode="after")
    def validate_debian_package_and_kconfigs_options_exclusive(self) -> Self:
        """Enforce debian_package and kconfigs options are exclusive"""
        debian = self.kernel_ubuntu_debian_package

        if debian and self.kernel_kconfigs:
            emit.warning(
                "'kernel-kconfigs' will be ignored when 'kernel-ubuntu-debian-package' is set",
            )
        return self

    @pydantic.model_validator(mode="after")
    def validate_debian_and_binary_exclusive(self) -> Self:
        """Enforce debian_package and binary_package options are mutually exclusive."""
        debian = self.kernel_ubuntu_debian_package
        binary = self.kernel_ubuntu_binary_package

        if binary and debian:
            raise errors.PartsError(
                brief="cannot use 'kernel-ubuntu-binary-package' and 'kernel-ubuntu-debian-package' at the same time",
                resolution="Choose one build method: set either 'kernel-ubuntu-binary-package' or 'kernel-ubuntu-debian-package', not both",
            )
        return self

    @pydantic.model_validator(mode="after")
    def validate_binary_and_tools_exclusive(self) -> Self:
        """Enforce binary_package and tools options are mutually exclusive"""
        binary = self.kernel_ubuntu_binary_package
        if binary and self.kernel_tools:
            raise errors.PartsError(
                brief="'kernel-tools' cannot be used with 'kernel-ubuntu-binary-package'",
                resolution="Add the required tools to 'stage-packages' instead",
            )
        return self

    @pydantic.model_validator(mode="after")
    def validate_debian_dkms_requires_debian_package(self) -> Self:
        """Enforce kernel_ubuntu_debian_dkms requires kernel_ubuntu_debian_package."""
        if self.kernel_ubuntu_debian_dkms and not self.kernel_ubuntu_debian_package:
            raise errors.PartsError(
                brief="'kernel-ubuntu-debian-dkms' requires 'kernel-ubuntu-debian-package' to be enabled",
                resolution="Set 'kernel-ubuntu-debian-package: true' or build the module as out-of-tree in the usual way",
            )
        return self


class KernelPlugin(plugins.Plugin):
    """Plugin class implementing kernel build functionality."""

    properties_class = KernelPluginProperties

    def __init__(
        self, *, properties: plugins.PluginProperties, part_info: infos.PartInfo
    ) -> None:
        super().__init__(properties=properties, part_info=part_info)
        self.options = cast(KernelPluginProperties, self._options)

    @property
    def out_of_source_build(self):
        """Return whether the plugin performs out-of-source-tree builds."""
        return True

    @override
    def get_pull_commands(self) -> list[str]:
        commands = []
        pkg = "linux"
        branch = self.options.kernel_ubuntu_abinumber
        flavour = self.options.kernel_ubuntu_kconfigflavour
        release = self.options.kernel_ubuntu_release_name
        binary = self.options.kernel_ubuntu_binary_package

        if binary:
            return super().get_pull_commands()

        if flavour != "generic":
            pkg = f"linux-{flavour}"

        if release:
            # Check for a valid URL
            # Launchpad will return "Invalid OpenID transaction" if a repository
            # doesn't exist, rather than curl failing. This string check is beholden to
            # Launchpad remaining consistent in their handling.
            commands.extend(
                [
                    "for name in canonical-kernel ubuntu-kernel; do",
                    f'team_url="https://git.launchpad.net/~$name/ubuntu/+source/{pkg}/+git/{release}"',
                    'test_url="$(curl -fsSL "$team_url")"',
                    'if [ -n "$test_url" ] && [ "$test_url" != "Invalid OpenID transaction" ]; then',
                    'actual_url="$team_url"',
                    "fi",
                    "done",
                ]
            )
            commands.extend(
                [
                    "git init",
                    'git remote add origin "$actual_url"',
                    f"git fetch --depth 1 origin {branch}",
                    "git checkout FETCH_HEAD",
                ]
            )

            return commands

        return super().get_pull_commands()

    @override
    def get_build_snaps(self) -> set[str]:
        return set()

    @override
    def get_build_packages(self) -> set[str]:
        base = self._part_info.base
        target_arch = self._part_info.target_arch
        target_arch_triplet = self._part_info.arch_triplet_build_for

        # We should install gcc for the target, but the x86_64 gcc package is named
        # differently from its triplet
        gcc_arch = target_arch_triplet
        match target_arch_triplet:
            case "x86_64-linux-gnu":
                gcc_arch = "x86-64-linux-gnu"

        build_packages = {
            "bc",
            "binutils",
            "bison",
            "cmake",
            "cpio",
            "cryptsetup",
            "debhelper",
            "fakeroot",
            "flex",
            "gawk",
            f"gcc-{gcc_arch}",
            "kmod",
            "kpartx",
            "libelf-dev",
            "libssl-dev",
            "lz4",
            "systemd",
            "xz-utils",
            "zstd",
        }

        # Rust was introduced in 23.04
        if base != "core22":
            build_packages |= {
                "clang",
                "rustc",
                "libdw-dev",
                "llvm",
            }

        # bpftool requires libelf, zlib to build
        if "bpftool" in self.options.kernel_tools:
            build_packages |= {
                f"libelf-dev:{target_arch}",
                f"zlib1g-dev:{target_arch}",
            }

        # cpupower requires libpci to build
        if "cpupower" in self.options.kernel_tools:
            build_packages |= {
                f"libpci-dev:{target_arch}",
            }

        if "perf" in self.options.kernel_tools:
            build_packages |= {
                f"libtraceevent-dev:{target_arch}",
                f"libiberty-dev:{target_arch}",
            }

        if {"bpftool", "perf"} & set(self.options.kernel_tools):
            build_packages |= {
                "libcap-dev",
            }

        # binary-perarch uses rsync to copy the source tree to a tools build dir
        if self.options.kernel_ubuntu_debian_package and self.options.kernel_tools:
            build_packages |= {
                "rsync",
            }

        if self.options.kernel_ubuntu_debian_dkms:
            build_packages |= {
                "dkms",
            }

        return build_packages

    @override
    def get_build_environment(self) -> dict[str, str]:
        kernel_arch = KERNEL_ARCH_FROM_SNAP_ARCH[self._part_info.target_arch]

        kernel_target = "modules"

        if kernel_arch != "x86":
            kernel_target = "modules dtbs"

        # The default kernel image is a compressed one
        # This is the default image name for amd64 and s390x
        kernel_image = "bzImage"
        # Choose a different kernel_image name based on the target architecture
        match kernel_arch:
            case "arm" | "powerpc":
                kernel_image = "zImage"
            case "arm64" | "riscv":
                kernel_image = "Image"

        return {
            "CROSS": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "CROSS_COMPILE": "${CRAFT_ARCH_TRIPLET_BUILD_FOR}-",
            "ARCH": kernel_arch,
            "KERNEL_IMAGE": kernel_image,
            "KERNEL_TARGET": kernel_target,
        }

    @override
    def get_build_commands(self) -> list[str]:
        kdefconfig = self.options.kernel_kdefconfig
        binary = self.options.kernel_ubuntu_binary_package
        debian = self.options.kernel_ubuntu_debian_package
        abinumber = self.options.kernel_ubuntu_abinumber
        release_name = self.options.kernel_ubuntu_release_name
        kconfigflavour = self.options.kernel_ubuntu_kconfigflavour

        if kdefconfig != ["defconfig"]:
            kconfigflavour = ""

        if (binary or debian) and kconfigflavour == "":
            kconfigflavour = "generic"

        if abinumber == "master-next":
            abinumber = ""

        release_name = release_name or "None"

        return [
            " ".join(
                [
                    "$SNAP/lib/python3.12/site-packages/snapcraft/parts/plugins/kernel_build.sh",
                    f"kernel-kdefconfig='{' '.join(self.options.kernel_kdefconfig)}'",
                    f"kernel-kconfigs='{','.join(self.options.kernel_kconfigs)}'",
                    f"kernel-tools='{' '.join(sorted(self.options.kernel_tools))}'",
                    f"kernel-ubuntu-kconfigflavour={kconfigflavour}",
                    f"kernel-ubuntu-release-name={release_name}",
                    f"kernel-ubuntu-abinumber={abinumber}",
                    f"kernel-ubuntu-binary-package={self.options.kernel_ubuntu_binary_package}",
                    f"kernel-ubuntu-debian-package={self.options.kernel_ubuntu_debian_package}",
                    f"kernel-ubuntu-debian-dkms='{' '.join(self.options.kernel_ubuntu_debian_dkms)}'",
                ]
            )
        ]
