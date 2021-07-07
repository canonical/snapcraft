# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

import fileinput
import functools
import logging
import os
import pathlib
import re
import subprocess
import sys
import tempfile
from typing import Dict, List, Optional, Set, Tuple  # noqa: F401

from xdg import BaseDirectory

from snapcraft import file_utils
from snapcraft.internal.indicators import is_dumb_terminal

from . import errors
from ._base import BaseRepo, get_pkg_name_parts
from .deb_package import DebPackage

if sys.platform == "linux":
    # Ensure importing works on non-Linux.
    from .apt_cache import AptCache

logger = logging.getLogger(__name__)

_DEB_CACHE_DIR: pathlib.Path = pathlib.Path(
    BaseDirectory.save_cache_path("snapcraft", "download")
)
_STAGE_CACHE_DIR: pathlib.Path = pathlib.Path(
    BaseDirectory.save_cache_path("snapcraft", "stage-packages")
)

_HASHSUM_MISMATCH_PATTERN = re.compile(r"(E:Failed to fetch.+Hash Sum mismatch)+")
_DEFAULT_FILTERED_STAGE_PACKAGES: List[str] = [
    "adduser",
    "apt",
    "apt-utils",
    "base-files",
    "base-passwd",
    "bash",
    "bsdutils",
    "coreutils",
    "dash",
    "debconf",
    "debconf-i18n",
    "debianutils",
    "diffutils",
    "dmsetup",
    "dpkg",
    "e2fslibs",
    "e2fsprogs",
    "file",
    "findutils",
    "gcc-4.9-base",
    "gcc-5-base",
    "gnupg",
    "gpgv",
    "grep",
    "gzip",
    "hostname",
    "init",
    "initscripts",
    "insserv",
    "libacl1",
    "libapparmor1",
    "libapt",
    "libapt-inst1.5",
    "libapt-pkg4.12",
    "libattr1",
    "libaudit-common",
    "libaudit1",
    "libblkid1",
    "libbz2-1.0",
    "libc-bin",
    "libc6",
    "libcap2",
    "libcap2-bin",
    "libcomerr2",
    "libcryptsetup4",
    "libdb5.3",
    "libdebconfclient0",
    "libdevmapper1.02.1",
    "libgcc1",
    "libgcrypt20",
    "libgpg-error0",
    "libgpm2",
    "libkmod2",
    "liblocale-gettext-perl",
    "liblzma5",
    "libmagic1",
    "libmount1",
    "libncurses5",
    "libncursesw5",
    "libpam-modules",
    "libpam-modules-bin",
    "libpam-runtime",
    "libpam0g",
    "libpcre3",
    "libprocps3",
    "libreadline6",
    "libselinux1",
    "libsemanage-common",
    "libsemanage1",
    "libsepol1",
    "libslang2",
    "libsmartcols1",
    "libss2",
    "libstdc++6",
    "libsystemd0",
    "libtext-charwidth-perl",
    "libtext-iconv-perl",
    "libtext-wrapi18n-perl",
    "libtinfo5",
    "libudev1",
    "libusb-0.1-4",
    "libustr-1.0-1",
    "libuuid1",
    "locales",
    "login",
    "lsb-base",
    "makedev",
    "manpages",
    "manpages-dev",
    "mawk",
    "mount",
    "multiarch-support",
    "ncurses-base",
    "ncurses-bin",
    "passwd",
    "perl-base",
    "procps",
    "readline-common",
    "sed",
    "sensible-utils",
    "systemd",
    "systemd-sysv",
    "sysv-rc",
    "sysvinit-utils",
    "tar",
    "tzdata",
    "ubuntu-keyring",
    "udev",
    "util-linux",
    "zlib1g",
]


IGNORE_FILTERS: Dict[str, Set[str]] = {
    "core20": {
        "python3-attr",
        "python3-blinker",
        "python3-certifi",
        "python3-cffi-backend",
        "python3-chardet",
        "python3-configobj",
        "python3-cryptography",
        # Rely on setuptools installed by plugin or found in base, unless
        # explicitly requested.
        # "python3-distutils"
        "python3-idna",
        "python3-importlib-metadata",
        "python3-jinja2",
        "python3-json-pointer",
        "python3-jsonpatch",
        "python3-jsonschema",
        "python3-jwt",
        "python3-lib2to3",
        "python3-markupsafe",
        # Provides /usr/bin/python3, don't bring in unless explicitly requested.
        # "python3-minimal"
        "python3-more-itertools",
        "python3-netifaces",
        "python3-oauthlib",
        # Rely on version brought in by setuptools, unless explicitly requested.
        # "python3-pkg-resources"
        "python3-pyrsistent",
        "python3-pyudev",
        "python3-requests",
        "python3-requests-unixsocket",
        "python3-serial",
        # Rely on version installed by plugin or found in base, unless
        # explicitly requested.
        # "python3-setuptools"
        "python3-six",
        "python3-urllib3",
        "python3-urwid",
        "python3-yaml",
        "python3-zipp",
    }
}


@functools.lru_cache(maxsize=256)
def _run_dpkg_query_search(file_path: str) -> str:
    try:
        output = (
            subprocess.check_output(
                ["dpkg-query", "-S", os.path.join(os.path.sep, file_path)],
                stderr=subprocess.STDOUT,
                env=dict(LANG="C.UTF-8"),
            )
            .decode()
            .strip()
        )
    except subprocess.CalledProcessError as call_error:
        logger.debug(
            "Error finding package for {}: {}".format(file_path, str(call_error))
        )
        raise errors.FileProviderNotFound(file_path=file_path) from call_error

    # Remove diversions
    provides_output = [p for p in output.splitlines() if not p.startswith("diversion")][
        0
    ]
    return provides_output.split(":")[0]


@functools.lru_cache(maxsize=256)
def _run_dpkg_query_list_files(package_name: str) -> Set[str]:
    output = (
        subprocess.check_output(["dpkg", "-L", package_name])
        .decode(sys.getfilesystemencoding())
        .strip()
        .split()
    )

    return {i for i in output if ("lib" in i and os.path.isfile(i))}


def _get_dpkg_list_path(base: str) -> pathlib.Path:
    return pathlib.Path(f"/snap/{base}/current/usr/share/snappy/dpkg.list")


def _get_filtered_stage_package_names(
    *, base: str, package_list: List[DebPackage]
) -> Set[str]:
    """Get filtered packages by name only - no version or architectures."""
    manifest_packages = [p.name for p in get_packages_in_base(base=base)]
    stage_packages = [p.name for p in package_list]

    return (
        set(manifest_packages) - set(stage_packages) - IGNORE_FILTERS.get(base, set())
    )


def get_packages_in_base(*, base: str) -> List[DebPackage]:
    # We do not want to break what we already have.
    if base in ("core", "core16", "core18"):
        return [DebPackage.from_unparsed(p) for p in _DEFAULT_FILTERED_STAGE_PACKAGES]

    base_package_list_path = _get_dpkg_list_path(base)
    if not base_package_list_path.exists():
        return list()

    # Lines we care about in dpkg.list had the following format:
    # ii adduser 3.118ubuntu1 all add and rem
    package_list = list()
    with fileinput.input(str(base_package_list_path)) as fp:
        for line in fp:
            if not line.startswith("ii "):
                continue
            package = DebPackage.from_unparsed(line.split()[1])
            package_list.append(package)

    return package_list


class Ubuntu(BaseRepo):
    @classmethod
    def get_package_libraries(cls, package_name: str) -> Set[str]:
        return _run_dpkg_query_list_files(package_name)

    @classmethod
    def get_package_for_file(cls, file_path: str) -> str:
        return _run_dpkg_query_search(file_path)

    @classmethod
    def get_packages_for_source_type(cls, source_type):
        if source_type == "bzr":
            packages = {"bzr"}
        elif source_type == "git":
            packages = {"git"}
        elif source_type == "tar":
            packages = {"tar"}
        elif source_type == "hg" or source_type == "mercurial":
            packages = {"mercurial"}
        elif source_type == "subversion" or source_type == "svn":
            packages = {"subversion"}
        elif source_type == "rpm2cpio":
            packages = {"rpm2cpio"}
        elif source_type == "7zip":
            packages = {"p7zip-full"}
        else:
            packages = set()

        return packages

    @classmethod
    def refresh_build_packages(cls) -> None:
        try:
            cmd = ["sudo", "--preserve-env", "apt-get", "update"]
            logger.debug(f"Executing: {cmd!r}")
            subprocess.check_call(cmd)
        except subprocess.CalledProcessError as call_error:
            raise errors.CacheUpdateFailedError(
                "failed to run apt update"
            ) from call_error

    @classmethod
    def _check_if_all_packages_installed(cls, package_names: List[str]) -> bool:
        """Check if all given packages are installed.

        Will check versions if using <pkg_name>=<pkg_version> syntax parsed by
        get_pkg_name_parts().  Used as an optimization to skip installation
        and cache refresh if dependencies are already satisfied.

        :return True if _all_ packages are installed (with correct versions).
        """

        with AptCache() as apt_cache:
            for package in package_names:
                pkg_name, pkg_version = get_pkg_name_parts(package)
                installed_version = apt_cache.get_installed_version(
                    pkg_name, resolve_virtual_packages=True
                )

                if installed_version is None or (
                    pkg_version is not None and installed_version != pkg_version
                ):
                    return False

        return True

    @classmethod
    def _get_packages_marked_for_installation(
        cls, package_names: List[str]
    ) -> List[Tuple[str, str]]:
        with AptCache() as apt_cache:
            try:
                apt_cache.mark_packages(set(package_names))
            except errors.PackageNotFoundError as error:
                raise errors.BuildPackageNotFoundError(error.package_name)

            return apt_cache.get_packages_marked_for_installation()

    @classmethod
    def install_build_packages(cls, package_names: List[str]) -> List[str]:
        """Install packages on the host required to build.

        :param package_names: a list of package names to install.
        :type package_names: a list of strings.
        :return: a list with the packages installed and their versions.
        :rtype: list of strings.
        :raises snapcraft.repo.errors.BuildPackageNotFoundError:
            if one of the packages was not found.
        :raises snapcraft.repo.errors.PackageBrokenError:
            if dependencies for one of the packages cannot be resolved.
        :raises snapcraft.repo.errors.BuildPackagesNotInstalledError:
            if installing the packages on the host failed.
        """
        install_required = False
        package_names = sorted(package_names)

        logger.debug(f"Requested build-packages: {package_names!r}")

        # Ensure we have an up-to-date cache first if we will have to
        # install anything.
        if not cls._check_if_all_packages_installed(package_names):
            install_required = True
            cls.refresh_build_packages()

        marked_packages = cls._get_packages_marked_for_installation(package_names)
        packages = [f"{name}={version}" for name, version in sorted(marked_packages)]

        if install_required:
            cls._install_packages(packages)
        else:
            logger.debug(f"Requested build-packages already installed: {packages!r}")

        return packages

    @classmethod
    def _install_packages(cls, package_names: List[str]) -> None:
        logger.info("Installing build dependencies: %s", " ".join(package_names))
        env = os.environ.copy()
        env.update(
            {
                "DEBIAN_FRONTEND": "noninteractive",
                "DEBCONF_NONINTERACTIVE_SEEN": "true",
                "DEBIAN_PRIORITY": "critical",
            }
        )

        apt_command = [
            "sudo",
            "--preserve-env",
            "apt-get",
            "--no-install-recommends",
            "-y",
            "--allow-downgrades",
        ]
        if not is_dumb_terminal():
            apt_command.extend(["-o", "Dpkg::Progress-Fancy=1"])
        apt_command.append("install")

        try:
            subprocess.check_call(apt_command + package_names, env=env)
        except subprocess.CalledProcessError:
            raise errors.BuildPackagesNotInstalledError(packages=package_names)

        versionless_names = [get_pkg_name_parts(p)[0] for p in package_names]
        try:
            subprocess.check_call(
                ["sudo", "apt-mark", "auto"] + versionless_names, env=env
            )
        except subprocess.CalledProcessError as e:
            logger.warning(
                "Impossible to mark packages as auto-installed: {}".format(e)
            )

    @classmethod
    def fetch_stage_packages(
        cls,
        *,
        package_names: List[str],
        base: str,
        stage_packages_path: pathlib.Path,
        target_arch: str,
    ) -> List[str]:
        logger.debug(f"Requested stage-packages: {sorted(package_names)!r}")

        installed: Set[str] = set()

        package_list = [DebPackage.from_unparsed(name) for name in package_names]
        filtered_names = _get_filtered_stage_package_names(
            base=base, package_list=package_list
        )

        stage_packages_path.mkdir(exist_ok=True)
        with AptCache(
            stage_cache=_STAGE_CACHE_DIR, stage_cache_arch=target_arch
        ) as apt_cache:
            apt_cache.update()
            apt_cache.mark_packages(set(package_names))
            apt_cache.unmark_packages(filtered_names)
            for pkg_name, pkg_version, dl_path in apt_cache.fetch_archives(
                _DEB_CACHE_DIR
            ):
                logger.debug(f"Extracting stage package: {pkg_name}")
                installed.add(f"{pkg_name}={pkg_version}")
                file_utils.link_or_copy(
                    str(dl_path), str(stage_packages_path / dl_path.name)
                )

        return sorted(installed)

    @classmethod
    def unpack_stage_packages(
        cls, *, stage_packages_path: pathlib.Path, install_path: pathlib.Path
    ) -> None:
        stage_packages = stage_packages_path.glob("*.deb")

        if not stage_packages:
            return

        for pkg_path in stage_packages:
            with tempfile.TemporaryDirectory(suffix="deb-extract") as extract_dir:
                # Extract deb package.
                cls._extract_deb(pkg_path, extract_dir)
                # Mark source of files.
                marked_name = cls._extract_deb_name_version(pkg_path)
                cls._mark_origin_stage_package(extract_dir, marked_name)
                # Stage files to install_dir.
                file_utils.link_or_copy_tree(extract_dir, install_path.as_posix())
        cls.normalize(str(install_path))

    @classmethod
    def build_package_is_valid(cls, package_name) -> bool:
        with AptCache() as apt_cache:
            return apt_cache.is_package_valid(package_name)

    @classmethod
    def is_package_installed(cls, package_name) -> bool:
        with AptCache() as apt_cache:
            return apt_cache.get_installed_version(package_name) is not None

    @classmethod
    def get_installed_packages(cls) -> List[str]:
        with AptCache() as apt_cache:
            return [
                f"{pkg_name}={pkg_version}"
                for pkg_name, pkg_version in apt_cache.get_installed_packages().items()
            ]

    @classmethod
    def _extract_deb_name_version(cls, deb_path: pathlib.Path) -> str:
        try:
            output = subprocess.check_output(
                ["dpkg-deb", "--show", "--showformat=${Package}=${Version}", deb_path]
            )
        except subprocess.CalledProcessError:
            raise errors.UnpackError(deb_path)

        return output.decode().strip()

    @classmethod
    def _extract_deb(cls, deb_path: pathlib.Path, extract_dir: str) -> None:
        """Extract deb and return `<package-name>=<version>`."""
        try:
            subprocess.check_call(["dpkg-deb", "--extract", deb_path, extract_dir])
        except subprocess.CalledProcessError:
            raise errors.UnpackError(deb_path)
