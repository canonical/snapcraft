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
import logging
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple  # noqa: F401

import apt
from xdg import BaseDirectory

from snapcraft import file_utils
from snapcraft.internal import os_release
from snapcraft.internal.errors import OsReleaseCodenameError
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
def _run_dpkg_query_s(file_path: str) -> str:
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


class Ubuntu(BaseRepo):
    _cache_dir: str = BaseDirectory.save_cache_path("snapcraft", "download")
    _cache: apt.Cache = apt.Cache()

    @classmethod
    def _refresh_cache(cls) -> None:
        if cls._cache:
            cls._cache.close()
        cls._cache = apt.Cache()

    @classmethod
    def get_package_libraries(cls, package_name: str) -> Set[str]:
        output = (
            subprocess.check_output(["dpkg", "-L", package_name])
            .decode(sys.getfilesystemencoding())
            .strip()
            .split()
        )

        return {i for i in output if ("lib" in i and os.path.isfile(i))}

    @classmethod
    def get_package_for_file(cls, file_path: str) -> str:
        return _run_dpkg_query_s(file_path)

    @classmethod
    def get_packages_for_source_type(cls, source_type: str) -> Set[str]:
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
    def refresh(cls) -> None:
        try:
            subprocess.check_call(["sudo", "--preserve-env", "apt-get", "update"])
        except subprocess.CalledProcessError as call_error:
            raise errors.CacheUpdateFailedError(
                "failed to run apt update"
            ) from call_error

        cls._refresh_cache()

    @classmethod
    def install_build_packages(cls, package_names: Set[str]) -> List[str]:
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

        cls._install_packages(sorted(package_names))

        return [
            f"{name}={cls._get_installed_package_version(name)}"
            for name in package_names
        ]

    @classmethod
    def _install_packages(cls, package_names: List[str]) -> None:
        logger.info(f"Installing build dependencies: {package_names}")
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
    def _resolve_virtual_package(cls, package_name: str) -> str:
        if cls._cache.is_virtual_package(package_name):
            package_name = cls._cache.get_providing_packages(package_name)[0]
        return package_name

    @classmethod
    def _get_cached_package(cls, package_name: str) -> apt.package.Package:
        # XXX: Chop of ":any" ??  Where would this come from?
        if package_name.endswith(":any"):
            package_name = package_name[:-4]

        # Resolve virtual package, if it is one.
        package_name = cls._resolve_virtual_package(package_name)

        return cls._cache[package_name]

    @classmethod
    def _mark_package_dependencies(
        cls,
        *,
        package: apt.package.Package,
        marked_packages: Dict[str, apt.package.Version],
        skipped_blacklisted: Set[str],
        skipped_essential: Set[str],
    ) -> None:
        if package.name in marked_packages:
            # already marked, ignore.
            return

        if package.name in _DEFAULT_FILTERED_STAGE_PACKAGES:
            skipped_blacklisted.add(package.name)
            return

        if package.candidate.priority == "essential":
            skipped_essential.add(package.name)
            return

        # We have to mark it first or risk recursion depth exceeded...
        marked_packages[package.name] = package.versions[0]

        for pkg_dep in package.candidate.dependencies:
            dep_pkg = pkg_dep.target_versions[0].package
            cls._mark_package_dependencies(
                package=dep_pkg,
                marked_packages=marked_packages,
                skipped_blacklisted=skipped_blacklisted,
                skipped_essential=skipped_essential,
            )

    @classmethod
    def install_stage_packages(
        cls, *, package_names: List[str], install_dir: str
    ) -> List[str]:
        marked_packages: Dict[str, apt.package.Version] = dict()
        skipped_blacklisted: Set[str] = set()
        skipped_essential: Set[str] = set()

        for package_name in package_names:
            package = cls._get_cached_package(package_name)
            cls._mark_package_dependencies(
                package=package,
                marked_packages=marked_packages,
                skipped_blacklisted=skipped_blacklisted,
                skipped_essential=skipped_essential,
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
            dl_path = pkg_version.fetch_binary(cls._cache_dir)

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
    def is_package_installed(cls, package_name: str) -> bool:
        if package_name not in cls._cache:
            return False
        return cls._cache[package_name].installed

    @classmethod
    def get_installed_packages(cls) -> List[str]:
        installed_packages = []
        for package in cls._cache:
            if package.installed:
                installed_packages.append(
                    "{}={}".format(package.name, package.installed.version)
                )
        return installed_packages

    @classmethod
    def _get_installed_package_version(cls, package_name: str) -> Optional[str]:
        try:
            package = cls._cache[package_name]
        except KeyError:
            return None

        return package.installed.version if package.installed else None

    @classmethod
    def is_valid(cls, package_name: str) -> bool:
        return package_name in cls._cache

    @classmethod
    def _extract_deb(cls, deb_path: str, extract_dir: str) -> None:
        """Extract deb and return `<package-name>=<version>`."""
        try:
            subprocess.check_call(["dpkg-deb", "--extract", deb_path, extract_dir])
        except subprocess.CalledProcessError:
            raise errors.UnpackError(deb_path)

    @classmethod
    def _apt_key_decoder_ring(cls, output: bytes) -> str:
        """apt-key can have some cryptic messages.  This helps improve them."""

        message: str = output.decode()
        message = message.replace(
            "Warning: apt-key output should not be parsed (stdout is not a terminal)",
            "",
        ).strip()

        return message

    @classmethod
    def _sudo_write_file(cls, dst_path: Path, content: bytes) -> None:
        """Workaround for writing to privileged files."""
        with tempfile.NamedTemporaryFile() as src_f:
            src_f.write(content)
            src_f.flush()

            try:
                command = [
                    "sudo",
                    "install",
                    "-o",
                    "root",
                    "-g",
                    "root",
                    "-m",
                    "0644",
                    src_f.name,
                    str(dst_path),
                ]
                subprocess.check_call(command)
            except subprocess.CalledProcessError:
                raise RuntimeError(f"failed to run: {command}")

    @classmethod
    def add_gpg_keyring(cls, *, name: str, gpg_keyring_path: str) -> None:
        src_keyring = Path(gpg_keyring_path)
        dst_keyring = Path("/etc", "apt", "trusted.gpg.d", name + ".gpg")

        cls._sudo_write_file(dst_path=dst_keyring, content=src_keyring.read_bytes())
        logger.debug(f"Installed gpg keyring {dst_keyring}.")

    @classmethod
    def add_source(cls, *, name: str, source_line: str) -> None:
        """Add deb source line. Supports formatting tag for {os_codename}."""

        conf = Path("/etc", "apt", "sources.list.d", name + ".list")

        try:
            os_codename = os_release.OsRelease().version_codename()
        except OsReleaseCodenameError:
            raise RuntimeError("unsupported operating system")

        source_line = source_line.format(os_codename=os_codename)

        cls._sudo_write_file(dst_path=conf, content=source_line.encode())
        cls.refresh()

        logger.debug(f"Installed apt repository in {conf}: {source_line}")
