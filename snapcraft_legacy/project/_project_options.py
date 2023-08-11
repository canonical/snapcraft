# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2019, 2023 Canonical Ltd
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
import multiprocessing
import os
import platform
import sys
from typing import Dict, List, Optional, Set, Union

from snapcraft_legacy import file_utils
from snapcraft_legacy.internal import common, errors, os_release

logger = logging.getLogger(__name__)


_ARCH_TRANSLATIONS = {
    "aarch64": {
        "kernel": "arm64",
        "deb": "arm64",
        "uts_machine": "aarch64",
        "cross-compiler-prefix": "aarch64-linux-gnu-",
        "cross-build-packages": ["gcc-aarch64-linux-gnu", "libc6-dev-arm64-cross"],
        "triplet": "aarch64-linux-gnu",
        "core-dynamic-linker": "lib/ld-linux-aarch64.so.1",
    },
    "arm64": {
        "kernel": "arm64",
        "deb": "arm64",
        "uts_machine": "aarch64",
        "cross-compiler-prefix": "aarch64-linux-gnu-",
        "cross-build-packages": ["gcc-aarch64-linux-gnu", "libc6-dev-arm64-cross"],
        "triplet": "aarch64-linux-gnu",
        "core-dynamic-linker": "lib/ld-linux-aarch64.so.1",
    },
    "armv7l": {
        "kernel": "arm",
        "deb": "armhf",
        "uts_machine": "arm",
        "cross-compiler-prefix": "arm-linux-gnueabihf-",
        "cross-build-packages": ["gcc-arm-linux-gnueabihf", "libc6-dev-armhf-cross"],
        "triplet": "arm-linux-gnueabihf",
        "core-dynamic-linker": "lib/ld-linux-armhf.so.3",
    },
    "i686": {
        "kernel": "x86",
        "deb": "i386",
        "uts_machine": "i686",
        "triplet": "i386-linux-gnu",
    },
    "ppc": {
        "kernel": "powerpc",
        "deb": "powerpc",
        "uts_machine": "powerpc",
        "cross-compiler-prefix": "powerpc-linux-gnu-",
        "cross-build-packages": ["gcc-powerpc-linux-gnu", "libc6-dev-powerpc-cross"],
        "triplet": "powerpc-linux-gnu",
    },
    "ppc64le": {
        "kernel": "powerpc",
        "deb": "ppc64el",
        "uts_machine": "ppc64el",
        "cross-compiler-prefix": "powerpc64le-linux-gnu-",
        "cross-build-packages": [
            "gcc-powerpc64le-linux-gnu",
            "libc6-dev-ppc64el-cross",
        ],
        "triplet": "powerpc64le-linux-gnu",
        "core-dynamic-linker": "lib64/ld64.so.2",
    },
    "riscv64": {
        "kernel": "riscv64",
        "deb": "riscv64",
        "uts_machine": "riscv64",
        "cross-compiler-prefix": "riscv64-linux-gnu-",
        "cross-build-packages": ["gcc-riscv64-linux-gnu", "libc6-dev-riscv64-cross"],
        "triplet": "riscv64-linux-gnu",
        "core-dynamic-linker": "lib/ld-linux-riscv64-lp64d.so.1",
    },
    "s390x": {
        "kernel": "s390",
        "deb": "s390x",
        "uts_machine": "s390x",
        "cross-compiler-prefix": "s390x-linux-gnu-",
        "cross-build-packages": ["gcc-s390x-linux-gnu", "libc6-dev-s390x-cross"],
        "triplet": "s390x-linux-gnu",
        "core-dynamic-linker": "lib/ld64.so.1",
    },
    "x86_64": {
        "kernel": "x86",
        "deb": "amd64",
        "uts_machine": "x86_64",
        "triplet": "x86_64-linux-gnu",
        "core-dynamic-linker": "lib64/ld-linux-x86-64.so.2",
    },
}


_32BIT_USERSPACE_ARCHITECTURE = {
    "aarch64": "armv7l",
    "armv8l": "armv7l",
    "ppc64le": "ppc",
    "x86_64": "i686",
}


_WINDOWS_TRANSLATIONS = {"AMD64": "x86_64"}


_HOST_CODENAME_FOR_BASE = {"core18": "bionic", "core20": "focal"}
_STATIC_BASES = ["bare"]

# TODO: just check the base.
_LINKER_VERSION_FOR_BASE = {"core20": "2.31", "core18": "2.27"}


def _get_platform_architecture():
    architecture = platform.machine()

    # Translate the windows architectures we know of to architectures
    # we can work with.
    if sys.platform == "win32":
        architecture = _WINDOWS_TRANSLATIONS.get(architecture)

    if platform.architecture()[0] == "32bit":
        userspace = _32BIT_USERSPACE_ARCHITECTURE.get(architecture)
        if userspace:
            architecture = userspace

    return architecture


class ProjectOptions:
    @property
    def parallel_build_count(self) -> int:
        try:
            build_count = len(os.sched_getaffinity(0))
        except AttributeError:
            # Fall back to multiprocessing.cpu_count()...
            try:
                build_count = multiprocessing.cpu_count()
            except NotImplementedError:
                logger.warning(
                    "Unable to determine CPU count; disabling parallel builds"
                )
                build_count = 1

        return build_count

    @property
    def is_cross_compiling(self):
        return self.__target_machine != self.__platform_arch

    @property
    def target_arch(self):
        """Returns the debian architecture of the run-on/build-for platform when
        cross-compiling. Returns the debian architecture of the build-on platform
        when not cross-compiling.
        """
        return self.__target_arch

    @property
    def cross_compiler_prefix(self):
        try:
            # cross-compilation of x86 32bit binaries on a x86_64 host is
            # possible by reusing the native toolchain - let Kbuild figure
            # it out by itself and pass down an empty cross-compiler-prefix
            # to start the build
            if self.__platform_arch == "x86_64" and self.__target_machine == "i686":
                return ""
            return self.__machine_info["cross-compiler-prefix"]
        except KeyError:
            raise errors.SnapcraftEnvironmentError(
                "Cross compilation not supported for target arch {!r}".format(
                    self.__target_machine
                )
            )

    @property
    def additional_build_packages(self):
        packages = []
        if self.is_cross_compiling:
            packages.extend(self.__machine_info.get("cross-build-packages", []))
        return packages

    @property
    def arch_triplet(self):
        """Returns the architecture triplet of the run-on platform when cross-compiling.
        Otherwise returns the architecture triplet of the build-on platform.
        """
        return self.__machine_info["triplet"]

    @property
    def deb_arch(self):
        """Returns the debian architecture of the run-on platform when cross-compiling.
        Otherwise returns the debian architecture of the build-on platform.
        """
        return self.__machine_info["deb"]

    @property
    def kernel_arch(self):
        return self.__machine_info["kernel"]

    @property
    def host_deb_arch(self):
        """Returns the debian architecture of the build-on platform."""
        return self.__host_info["deb"]

    @property
    def parts_dir(self) -> str:
        return self._parts_dir

    @property
    def stage_dir(self) -> str:
        return self._stage_dir

    @property
    def prime_dir(self) -> str:
        return self._prime_dir

    @property
    def debug(self):
        return self._debug

    @property
    def arch_build_on(self) -> str:
        """Returns the build-on architecture."""
        return self.__host_info["deb"]

    @property
    def arch_triplet_build_on(self) -> str:
        """Returns the build-on architecture."""
        return self.__host_info["triplet"]

    @property
    def arch_run_on(self) -> str:
        """Returns the run-on architecture."""
        return self.__run_on_info["deb"] if self.__run_on_info else None

    @property
    def arch_triplet_run_on(self) -> str:
        """Returns the run-on architecture."""
        return self.__run_on_info["triplet"] if self.__run_on_info else None

    def _set_run_on_info(
        self, target_deb_arch: str, architectures: List[Union[str, Dict[str, str]]]
    ) -> None:
        """Set machine info for the run-on platform, if possible.

        If `target_deb_arch` was provided for cross-compiling, then this architecture
        is used for the run-on platform. If `target_deb_arch` was not provided, the
        run-on platform will be determined from the architectures in the snapcraft.yaml.

        The run-on architecture cannot be determined for multi-arch builds. These are
        defined with a shorthand list `architectures: [arch1, arch2]` or when multiple
        run-on architectures are provided `run-on: [arch1, arch2]`.

        If no architectures are provided, the run-on platform will be set to the
        build-on platform. Finally, if the run-on architecture could not be determined
        then it will be set to None.

        :param target_deb_arch: the target architecture when cross-compiling
        :param architectures: the project's architecture data to process

        :returns: a dictionary of information about the run-on architecture or None
        """
        self.__run_on_info = None

        # if `target-arch` was provided, use it for the run-on arch
        if target_deb_arch:
            self.__run_on_info = _ARCH_TRANSLATIONS[self.__target_machine]
        # if no architectures were provided, set run-on arch to the build-on arch
        elif not architectures:
            self.__run_on_info = self.__host_info
        # else look for a run-on architecture from the snapcraft.yaml
        else:
            self.__run_on_info = self._process_architecture_data(architectures)

        if self.__run_on_info:
            logger.debug("Set run-on platform to %r", self.__run_on_info["deb"])
        else:
            logger.debug("Could not determine a run-on platform.")

    def _process_architecture_data(
        self, architectures: List[Union[str, Dict[str, str]]]
    ) -> Optional[Dict[str, str]]:
        """Process architecture data and attempt to determine the run-on architecture.

        This occurs before the architecture data is fully validated, so this function
        will not raise errors on invalid data. Instead, it will return None and let the
        validators raise an error for the user later on.

        :param architectures: the project's architecture data to process

        :returns: a dictionary of information about the run-on architecture or None
        """
        # if not a list, the validator will raise an error later
        if not isinstance(architectures, list):
            return None

        # if a list of strings was provided, then the architecture can be decoded
        if isinstance(architectures[0], str):
            return self._determine_run_on_architecture(architectures)

        # otherwise, parse through the 'build-on' and 'run-on' fields
        for item in architectures:
            arch_build_on = item.get("build-on")
            arch_run_on = item.get("run-on")
            if arch_build_on and self.__host_info["deb"] in arch_build_on:
                # run-on must be a scalar, not a multi-arch list
                if arch_run_on:
                    return self._determine_run_on_architecture(arch_run_on)
                # if there is no run-on, try to use the build-on field
                return self._determine_run_on_architecture(arch_build_on)
        return None

    def _determine_run_on_architecture(
        self, architectures: Union[str, List[str]]
    ) -> Optional[Dict[str, str]]:
        """Determine the run-on architecture.

        The run-on architecture can only be determined when a string or a single element
        list is provided. The run-on arch must also be a valid architecture.

        :param architectures: a single architecture or a list of architectures

        :returns: a dictionary of information about the run-on architecture or None
        """
        if isinstance(architectures, list):
            # convert single element list to string
            if len(architectures) == 1:
                architectures = architectures[0]
            # lists of architectures cannot be decoded
            else:
                logger.debug("Cannot set run-on info for multi-arch build")
                return None

        try:
            return _ARCH_TRANSLATIONS[_find_machine(architectures)]
        except errors.SnapcraftEnvironmentError:
            logger.debug(
                "Cannot set run-on info from unknown architectures %r", architectures
            )
            return None

    def __init__(
        self,
        target_deb_arch=None,
        debug=False,
        *,
        work_dir: str = None,
        architectures = None,
    ) -> None:

        # Here for backwards compatibility.
        project_dir = os.getcwd()
        if work_dir is None:
            work_dir = project_dir

        self._debug = debug

        self._parts_dir = os.path.join(work_dir, "parts")
        self._stage_dir = os.path.join(work_dir, "stage")
        self._prime_dir = os.path.join(work_dir, "prime")

        logger.debug("Parts dir {}".format(self._parts_dir))
        logger.debug("Stage dir {}".format(self._stage_dir))
        logger.debug("Prime dir {}".format(self._prime_dir))

        self._set_machine(target_deb_arch)
        self._set_run_on_info(target_deb_arch, architectures)

    def _get_content_snaps(self) -> Set[str]:
        """Temporary shim for unit tests using ProjectOptions
        where Project is really required.  Will be removed in
        future convergence work.
        """
        return set()

    def _get_provider_content_dirs(self) -> Set[str]:
        """Temporary shim for unit tests using ProjectOptions
        where Project is really required.  Will be removed in
        future convergence work.
        """
        return set()

    def _get_stage_packages_target_arch(self) -> str:
        """Stub for 'Project' interface for tests using ProjectOptions()."""
        return self.deb_arch

    def is_static_base(self, base: str) -> bool:
        """Return True if a base that is intended to be static is used.

        Static bases require all their necessary components to live within
        the snap.
        """
        return base in _STATIC_BASES

    def is_host_compatible_with_base(self, base: str) -> bool:
        """Determines if the host is compatible with the GLIBC of the base.

        The system should warn early on when building using a host that does
        not match the intended base, this mechanism here enables additional
        logic when that is ignored to determine built projects will actually
        run.

        :param str base: the base core snap to search for linker.
        :returns: True if there are no GLIBC incompatibilities with the chosen
                  build host, else it returns False.
        :rtype: bool
        """
        try:
            codename = os_release.OsRelease().version_codename()
        except errors.OsReleaseCodenameError:
            return False

        logger.debug("Running on {!r}".format(codename))

        # TODO: we should get rid of this check.
        return _HOST_CODENAME_FOR_BASE.get(base) == codename

    # This is private to not make the API public given that base
    # will be part of the new Project.
    def _get_linker_version_for_base(self, base: str) -> str:
        """Returns the linker version for base."""
        try:
            return _LINKER_VERSION_FOR_BASE[base]
        except KeyError:
            linker_file = os.path.basename(self.get_core_dynamic_linker(base))
            return file_utils.get_linker_version_from_file(linker_file)

    def get_core_dynamic_linker(self, base: str, expand: bool = True) -> str:
        """Returns the dynamic linker used for the targeted core.

        :param str base: the base core snap to search for linker.
        :param bool expand: expand the linker to the actual linker if True,
                            else the main entry point to the linker for the
                            projects architecture.
        :return: the absolute path to the linker
        :rtype: str
        :raises snapcraft_legacy.internal.errors.SnapcraftMissingLinkerInBaseError:
            if the linker cannot be found in the base.
        :raises snapcraft_legacy.internal.errors.SnapcraftEnvironmentError:
            if a loop is found while resolving the real path to the linker.
        """
        core_path = common.get_installed_snap_path(base)
        dynamic_linker_path = os.path.join(
            core_path,
            self.__machine_info.get("core-dynamic-linker", "lib/ld-linux.so.2"),
        )

        # return immediately if we do not need to expand
        if not expand:
            return dynamic_linker_path

        # We can't use os.path.realpath because any absolute symlinks
        # have to be interpreted relative to core_path, not the real
        # root.
        seen_paths = set()  # type: Set[str]
        while True:
            if dynamic_linker_path in seen_paths:
                raise errors.SnapcraftEnvironmentError(
                    "found symlink loop resolving dynamic linker path"
                )

            seen_paths.add(dynamic_linker_path)
            if not os.path.lexists(dynamic_linker_path):
                raise errors.SnapcraftMissingLinkerInBaseError(
                    base=base, linker_path=dynamic_linker_path
                )
            if not os.path.islink(dynamic_linker_path):
                return dynamic_linker_path

            link_contents = os.readlink(dynamic_linker_path)
            if os.path.isabs(link_contents):
                dynamic_linker_path = os.path.join(core_path, link_contents.lstrip("/"))
            else:
                dynamic_linker_path = os.path.join(
                    os.path.dirname(dynamic_linker_path), link_contents
                )

    def _set_machine(self, target_deb_arch):
        self.__platform_arch = _get_platform_architecture()
        if not target_deb_arch:
            self.__target_machine = self.__platform_arch
        else:
            self.__target_machine = _find_machine(target_deb_arch)
            logger.info("Setting target machine to {!r}".format(target_deb_arch))

        self.__machine_info = _ARCH_TRANSLATIONS[self.__target_machine]
        self.__host_info = _ARCH_TRANSLATIONS[self.__platform_arch]

        # Set target arch to match the host if unspecified.
        if target_deb_arch is None:
            self.__target_arch = self.__machine_info.get("deb")
        else:
            self.__target_arch = target_deb_arch


def _get_deb_arch(machine):
    return _ARCH_TRANSLATIONS[machine].get("deb", None)


def _find_machine(deb_arch):
    for machine in _ARCH_TRANSLATIONS:
        if _ARCH_TRANSLATIONS[machine].get("deb", "") == deb_arch:
            return machine
        elif _ARCH_TRANSLATIONS[machine].get("uts_machine", "") == deb_arch:
            return machine

    raise errors.SnapcraftEnvironmentError(
        "Cannot set machine from deb_arch {!r}".format(deb_arch)
    )
