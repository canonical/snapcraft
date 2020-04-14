# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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


import functools
import glob
import logging
import os
import re
import string
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple  # noqa: F401
from typing_extensions import Final

import apt
from xdg import BaseDirectory

from snapcraft import file_utils
from snapcraft.internal import os_release, repo
from snapcraft.internal.indicators import is_dumb_terminal

from . import errors
from ._base import BaseRepo

logger = logging.getLogger(__name__)

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


def _sudo_write_file(*, dst_path: Path, content: bytes) -> None:
    """Workaround for writing to privileged files."""
    with tempfile.NamedTemporaryFile() as src_f:
        src_f.write(content)
        src_f.flush()

        try:
            command = [
                "sudo",
                "install",
                "--owner=root",
                "--group=root",
                "--mode=0644",
                src_f.name,
                str(dst_path),
            ]
            subprocess.check_call(command)
        except subprocess.CalledProcessError:
            raise RuntimeError(f"failed to run: {command!r}")


class Ubuntu(BaseRepo):
    _SNAPCRAFT_INSTALLED_GPG_KEYRING: Final[
        str
    ] = "/etc/apt/trusted.gpg.d/snapcraft.gpg"
    _SNAPCRAFT_INSTALLED_SOURCES_LIST: Final[
        str
    ] = "/etc/apt/sources.list.d/snapcraft.list"

    _cache_dir: str = BaseDirectory.save_cache_path("snapcraft", "download")
    _cache: Optional[apt.Cache] = None

    @classmethod
    def _get_apt_cache(cls) -> apt.Cache:
        if cls._cache is None:
            cls._cache = apt.Cache()
        return cls._cache

    @classmethod
    def _refresh_cache(cls) -> None:
        if cls._cache is not None:
            cls._cache.close()
        cls._cache = apt.Cache()

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
            subprocess.check_call(["sudo", "--preserve-env", "apt-get", "update"])
        except subprocess.CalledProcessError as call_error:
            raise errors.CacheUpdateFailedError(
                "failed to run apt update"
            ) from call_error

        cls._refresh_cache()

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
        if not package_names:
            return list()

        # Make sure all packages are valid and remove already installed.
        install_packages = list()
        for name in sorted(package_names):
            name, version = repo.get_pkg_name_parts(name)

            try:
                package = cls._get_resolved_package(name)
            except errors.PackageNotFoundError:
                raise errors.BuildPackageNotFoundError(name)

            if name != package.name:
                logger.info(
                    f"virtual build-package {name!r} resolved to {package.name!r}"
                )

            # Reconstruct resolved package name, if version used.
            if version:
                name = f"{package.name}={version}"
            else:
                name = package.name

            if package.installed is None:
                install_packages.append(name)

        # Install packages, if any.
        if install_packages:
            cls._install_packages(install_packages)

        # Return installed packages with version info.
        return [
            f"{name}={cls._get_installed_package_version(name)}"
            for name in install_packages
        ]

    @classmethod
    def _get_installed_package_version(cls, package_name: str) -> Optional[str]:
        try:
            package = cls._get_apt_cache()[package_name]
        except KeyError:
            return None

        return package.installed.version if package.installed else None

    @classmethod
    def _install_packages(cls, package_names: List[str]) -> None:
        package_names.sort()
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
        ]
        if not is_dumb_terminal():
            apt_command.extend(["-o", "Dpkg::Progress-Fancy=1"])
        apt_command.append("install")

        try:
            subprocess.check_call(apt_command + package_names, env=env)
        except subprocess.CalledProcessError:
            raise errors.BuildPackagesNotInstalledError(packages=package_names)

        try:
            subprocess.check_call(["sudo", "apt-mark", "auto"] + package_names, env=env)
        except subprocess.CalledProcessError as e:
            logger.warning(
                "Impossible to mark packages as auto-installed: {}".format(e)
            )

        cls._refresh_cache()

    @classmethod
    def _get_resolved_package(cls, package_name: str) -> apt.package.Package:
        # Strip ":any" that may come from dependency names.
        if package_name.endswith(":any"):
            package_name = package_name[:-4]

        cache = cls._get_apt_cache()
        if cache.is_virtual_package(package_name):
            package = cache.get_providing_packages(package_name)[0]
            logger.debug(f"package {package_name!r} resolved to {package.name!r}")
            return package

        try:
            return cache[package_name]
        except KeyError:
            raise errors.PackageNotFoundError(package_name)

    @classmethod
    def _set_package_version(
        cls, package: apt.package.Package, version: Optional[str] = None
    ) -> None:
        """Set cadidate version to a specific version if available"""
        if version in package.versions:
            version = package.versions.get(version)
            package.candidate = version
        else:
            raise errors.PackageNotFoundError("{}={}".format(package.name, version))

    @classmethod
    def _mark_package_dependencies(
        cls,
        *,
        package: apt.package.Package,
        marked_packages: Dict[str, apt.package.Version],
        skipped_blacklisted: Set[str],
        skipped_essential: Set[str],
        unfiltered_packages: List[str],
    ) -> None:
        if package.name in marked_packages:
            # already marked, ignore.
            return

        # If package is not explicitly asked for in unfiltered packages,
        # we will filter out base packages using the base's manifest and
        # essential priority.
        if package.name not in unfiltered_packages:
            if cls._is_filtered_package(package.name):
                skipped_blacklisted.add(package.name)
                return

            if package.candidate.priority == "essential":
                skipped_essential.add(package.name)
                return

        # We have to mark it first or risk recursion depth exceeded...
        marked_packages[package.name] = package.candidate

        for pkg_dep in package.candidate.dependencies:
            dep_pkg = pkg_dep.target_versions[0].package
            cls._mark_package_dependencies(
                package=dep_pkg,
                marked_packages=marked_packages,
                skipped_blacklisted=skipped_blacklisted,
                skipped_essential=skipped_essential,
                unfiltered_packages=unfiltered_packages,
            )

    @classmethod
    def install_stage_packages(
        cls, *, package_names: List[str], install_dir: str
    ) -> List[str]:
        marked_packages: Dict[str, apt.package.Version] = dict()
        skipped_blacklisted: Set[str] = set()
        skipped_essential: Set[str] = set()

        # First scan all packages and set desired version, if specified.
        # We do this all at once in case it gets added as a dependency
        # along the way.
        for name in package_names:
            name, specified_version = repo.get_pkg_name_parts(name)

            package = cls._get_resolved_package(name)
            if name != package.name:
                logger.info(
                    f"virtual stage-package {name!r} resolved to {package.name!r}"
                )

            if specified_version:
                cls._set_package_version(package, specified_version)

        for name in package_names:
            name, _ = repo.get_pkg_name_parts(name)
            package = cls._get_resolved_package(name)
            cls._mark_package_dependencies(
                package=package,
                marked_packages=marked_packages,
                skipped_blacklisted=skipped_blacklisted,
                skipped_essential=skipped_essential,
                unfiltered_packages=package_names,
            )

        marked = sorted(marked_packages.keys())
        logger.debug(f"Marked staged packages: {marked!r}")

        if skipped_blacklisted:
            blacklisted = sorted(skipped_blacklisted)
            logger.debug(f"Skipping blacklisted packages: {blacklisted!r}")

        if skipped_essential:
            essential = sorted(skipped_essential)
            logger.debug(f"Skipping priority essential packages: {essential!r}")

        for pkg_name, pkg_version in marked_packages.items():
            try:
                dl_path = pkg_version.fetch_binary(cls._cache_dir)
            except apt.package.FetchError as e:
                raise errors.PackageFetchError(str(e))

            logger.debug(f"Extracting stage package: {pkg_name}")
            with tempfile.TemporaryDirectory() as temp_dir:
                # Extract deb package.
                cls._extract_deb(dl_path, temp_dir)

                # Mark source of files.
                marked_name = f"{pkg_name}:{pkg_version.version}"
                cls._mark_origin_stage_package(temp_dir, marked_name)

                # Stage files to install_dir.
                file_utils.link_or_copy_tree(temp_dir, install_dir)

        cls.normalize(install_dir)

        return [
            f"{pkg_name}={pkg_version}"
            for pkg_name, pkg_version in marked_packages.items()
        ]

    @classmethod
    def build_package_is_valid(cls, package_name):
        try:
            _ = cls._get_resolved_package(package_name)
        except errors.PackageNotFoundError:
            return False
        return True

    @classmethod
    def is_package_installed(cls, package_name):
        try:
            package = cls._get_resolved_package(package_name)
        except errors.PackageNotFoundError:
            return False

        return package.installed is not None

    @classmethod
    def get_installed_packages(cls) -> List[str]:
        installed_packages = []
        for package in cls._get_apt_cache():
            if package.installed is not None:
                installed_packages.append(
                    "{}={}".format(package.name, package.installed.version)
                )
        return installed_packages

    @classmethod
    def install_gpg_key(cls, gpg_key: str) -> None:
        cmd = [
            "sudo",
            "apt-key",
            "--keyring",
            cls._SNAPCRAFT_INSTALLED_GPG_KEYRING,
            "add",
            "-",
        ]
        try:
            subprocess.run(
                cmd,
                input=gpg_key.encode(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
            )
        except subprocess.CalledProcessError as error:
            raise errors.AptGPGKeyInstallError(output=error.output, gpg_key=gpg_key)

        logger.debug(f"Installed apt repository key:\n{gpg_key}")

    @classmethod
    def _get_snapcraft_installed_sources(cls) -> Set[str]:
        sources: Set[str] = set()

        installed_path = Path(cls._SNAPCRAFT_INSTALLED_SOURCES_LIST)
        if installed_path.exists():
            sources = set(installed_path.read_text().splitlines())

        return sources

    @classmethod
    def _set_snapcraft_installed_sources(cls, sources: Set[str]) -> None:
        installed_path = Path(cls._SNAPCRAFT_INSTALLED_SOURCES_LIST)
        sources_content = "\n".join(sorted(sources)) + "\n"
        _sudo_write_file(dst_path=installed_path, content=sources_content.encode())

    @classmethod
    def install_source(cls, source_line: str) -> None:
        """Add deb source line. Supports formatting tag for ${release}."""
        expanded_source = _format_sources_list(source_line)

        sources = cls._get_snapcraft_installed_sources()
        sources.add(expanded_source)

        cls._set_snapcraft_installed_sources(sources)
        cls.refresh_build_packages()

        logger.debug(f"Installed apt repository {expanded_source!r}")

    @classmethod
    def _is_filtered_package(cls, package_name: str) -> bool:
        # Filter out packages provided by the core snap.
        # TODO: use manifest found in core snap, if found at:
        # <core-snap>/usr/share/snappy/dpkg.list
        return package_name in _DEFAULT_FILTERED_STAGE_PACKAGES

    @classmethod
    def _extract_deb_name_version(cls, deb_path: str) -> str:
        try:
            output = subprocess.check_output(
                ["dpkg-deb", "--show", "--showformat=${Package}=${Version}", deb_path]
            )
        except subprocess.CalledProcessError:
            raise errors.UnpackError(deb_path)

        return output.decode().strip()

    @classmethod
    def _extract_deb(cls, deb_path: str, extract_dir: str) -> None:
        """Extract deb and return `<package-name>=<version>`."""
        try:
            subprocess.check_call(["dpkg-deb", "--extract", deb_path, extract_dir])
        except subprocess.CalledProcessError:
            raise errors.UnpackError(deb_path)


def _get_local_sources_list():
    sources_list = glob.glob("/etc/apt/sources.list.d/*.list")
    sources_list.append("/etc/apt/sources.list")

    sources = ""
    for source in sources_list:
        with open(source) as f:
            sources += f.read()

    return sources


def _format_sources_list(sources_list: str):
    release = os_release.OsRelease().version_codename()

    return string.Template(sources_list).substitute({"release": release})
