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
import gnupg
import io
import lazr.restfulclient.errors
import logging
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple  # noqa: F401
from typing_extensions import Final

import apt
from launchpadlib.launchpad import Launchpad
from xdg import BaseDirectory

from snapcraft import file_utils
from snapcraft.project._project_options import ProjectOptions
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


def _get_host_arch() -> str:
    return ProjectOptions().deb_arch


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
            cmd = ["sudo", "--preserve-env", "apt-get", "update"]
            logger.debug(f"Executing: {cmd!r}")
            subprocess.check_call(cmd)
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
        logger.debug(f"Requested build-packages: {sorted(package_names)!r}")

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
    def _package_name_parts(
        cls, package_name: str
    ) -> Tuple[str, Optional[str], Optional[str]]:
        split_version = package_name.split("=")
        if len(split_version) > 1:
            version: Optional[str] = split_version[1]
        else:
            version = None

        # Purge possible ":any" architecture tag, then split for arch.
        any_filtered = split_version[0].replace(":any", "")
        split_arch = any_filtered.split(":")
        if len(split_arch) > 1:
            arch: Optional[str] = split_arch[1]
        else:
            arch = None

        return (split_arch[0], arch, version)

    @classmethod
    def _get_package_for_arch(
        cls, *, package_name: str, target_arch: Optional[str]
    ) -> apt.Package:
        name, arch, version = cls._package_name_parts(package_name)
        logger.debug(f"Getting package {package_name!r} for {target_arch!r}.")

        cache = cls._get_apt_cache()

        if target_arch is not None and f"{name}:{target_arch}" in cache:
            # First check for explicit <package-name>:<arch>.
            package = cache[f"{name}:{target_arch}"]
        elif f"{name}:all" in cache:
            # Then check if package is :all in case it is not arch-specific.
            package = cache[f"{name}:all"]
        elif f"{name}:{arch}" in cache:
            # Then rely on the arch that was specified in the name.
            package = cache[f"{name}:{arch}"]
        elif name in cache:
            # Rely on the default selected by apt.Cache(), we'll check/warn
            # on the architecture below if we think it might be incorrect.
            package = cache[name]
        else:
            raise errors.PackageNotFoundError(package_name)

        if target_arch is not None and package.architecture() != target_arch:
            logger.debug(
                f"Possible incorrect architecture for package {name!r}. "
                f"Found architecture {package.architecture()!r}, "
                f"intended architecture is {target_arch!r}."
            )

        return package

    @classmethod
    def _get_resolved_package(
        cls, package_name: str, *, target_arch: Optional[str] = None
    ) -> apt.package.Package:
        name, arch, version = cls._package_name_parts(package_name)

        cache = cls._get_apt_cache()
        if cache.is_virtual_package(name) is True:
            name = cache.get_providing_packages(name)[0].name
            logger.info(f"Virtual package {package_name!r} resolved to {name!r}")

        return cls._get_package_for_arch(package_name=name, target_arch=target_arch)

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
        package_name: str,
        marked_packages: Dict[str, apt.package.Version],
        skipped_blacklisted: Set[str],
        skipped_essential: Set[str],
        unfiltered_packages: List[str],
        target_arch: Optional[str],
    ) -> None:
        # If we come across a package that explicitly requests a target arch
        # via <package-name>:<arch>, update target_arch as we are crossing
        # an architecture boundary.
        _, arch, _ = cls._package_name_parts(package_name)
        if arch:
            target_arch = arch

        package = cls._get_resolved_package(package_name, target_arch=target_arch)

        # Virtual packages may resolve to a foreign architecture. For
        # example: 'wine-devel-i386' resolved to 'wine-devel-i386:i386'
        # Use the resolved package's architecture as the target arch.
        if target_arch is None:
            target_arch = package.architecture()

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
            dep_name = pkg_dep.target_versions[0].package.name
            cls._mark_package_dependencies(
                package_name=dep_name,
                marked_packages=marked_packages,
                skipped_blacklisted=skipped_blacklisted,
                skipped_essential=skipped_essential,
                unfiltered_packages=unfiltered_packages,
                target_arch=target_arch,
            )

    @classmethod
    def install_stage_packages(
        cls, *, package_names: List[str], install_dir: str
    ) -> List[str]:
        marked_packages: Dict[str, apt.package.Version] = dict()
        skipped_blacklisted: Set[str] = set()
        skipped_essential: Set[str] = set()

        logger.debug(f"Requested stage-packages: {sorted(package_names)!r}")

        # Some projects will modify apt configuration directly.  To ensure
        # our cache is up-to-date, refresh the cache whenever stage-packages
        # are requested.
        cls._refresh_cache()

        # First scan all packages and set desired version, if specified.
        # We do this all at once in case it gets added as a dependency
        # along the way.
        for name in package_names:
            name, arch, version = cls._package_name_parts(name)
            package = cls._get_resolved_package(name, target_arch=arch)
            if version:
                cls._set_package_version(package, version)

        for name in package_names:
            logger.debug(f"Marking package dependencies for {name!r}.")
            name, arch, _ = cls._package_name_parts(name)

            cls._mark_package_dependencies(
                package_name=name,
                marked_packages=marked_packages,
                skipped_blacklisted=skipped_blacklisted,
                skipped_essential=skipped_essential,
                unfiltered_packages=package_names,
                target_arch=arch,
            )

        marked = sorted(marked_packages.keys())
        logger.debug(f"Installing stage-packages {marked!r} to {install_dir!r}")

        if skipped_blacklisted:
            blacklisted = sorted(skipped_blacklisted)
            logger.debug(f"Skipping blacklisted packages: {blacklisted!r}")

        if skipped_essential:
            essential = sorted(skipped_essential)
            logger.debug(f"Skipping priority essential packages: {essential!r}")

        # Install the package in reverse sorted manner.  This way, packages
        # that are cross-arch, e.g. "foo:i386", will get installed before the
        # native "foo".  In this manner, host-arch will will be the last
        # written file in case there are overlapping paths (e.g. i386 bins).
        # TODO: detect and warn when we are overwriting a staged file with
        # another one.
        for pkg_name, pkg_version in sorted(marked_packages.items(), reverse=True):
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
            f"{pkg_name}={pkg_version.version}"
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
    def _get_key_fingerprints(cls, key: str) -> List[str]:
        with tempfile.NamedTemporaryFile(suffix="keyring") as temp_file:
            return (
                gnupg.GPG(keyring=temp_file.name).import_keys(key_data=key).fingerprints
            )

    @classmethod
    def _is_key_id_installed(cls, key_id: str) -> bool:
        # Check if key is installed by attempting to export the key.
        # Unfortunately, apt-key does not exit with error, and
        # we have to do our best to parse the output.
        try:
            proc = subprocess.run(
                ["sudo", "apt-key", "export", key_id],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
            )
        except subprocess.CalledProcessError as error:
            # Export shouldn't exit with failure based on testing,
            # but assume the key is not installed and log a warning.
            logger.warning(f"Unexpected apt-key failure: {error.output}")
            return False

        apt_key_output = proc.stdout.decode()

        if "BEGIN PGP PUBLIC KEY BLOCK" in apt_key_output:
            return True

        if "nothing exported" in apt_key_output:
            return False

        # The two strings above have worked in testing, but if neither is
        # present for whatever reason, assume the key is not installed
        # and log a warning.
        logger.warning(f"Unexpected apt-key output: {apt_key_output}")
        return False

    @classmethod
    def _install_gpg_key(cls, *, key_id: str, key: str) -> None:
        cmd = [
            "sudo",
            "apt-key",
            "--keyring",
            cls._SNAPCRAFT_INSTALLED_GPG_KEYRING,
            "add",
            "-",
        ]
        try:
            logger.debug(f"Executing: {cmd!r}")
            env = os.environ.copy()
            env["LANG"] = "C.UTF-8"
            subprocess.run(
                cmd,
                input=key.encode(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
                env=env,
            )
        except subprocess.CalledProcessError as error:
            raise errors.AptGPGKeyInstallError(output=error.output.decode(), key=key)

        logger.debug(f"Installed apt repository key:\n{key}")

    @classmethod
    def install_gpg_key(cls, *, key_id: str, key: str) -> bool:
        if cls._is_key_id_installed(key_id):
            # Already installed, nothing to do.
            return False

        cls._install_gpg_key(key_id=key_id, key=key)
        return True

    @classmethod
    def _install_gpg_key_id_from_keyserver(
        cls, *, key_id: str, key_server: Optional[str] = None
    ) -> None:
        # Default to keyserver.ubuntu.com.
        if key_server is None:
            key_server = "keyserver.ubuntu.com"

        env = os.environ.copy()
        env["LANG"] = "C.UTF-8"

        cmd = [
            "sudo",
            "apt-key",
            "--keyring",
            cls._SNAPCRAFT_INSTALLED_GPG_KEYRING,
            "adv",
            "--keyserver",
            key_server,
            "--recv-keys",
            key_id,
        ]

        try:
            logger.debug(f"Executing: {cmd!r}")
            subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
                env=env,
            )
        except subprocess.CalledProcessError as error:
            raise errors.AptGPGKeyInstallError(
                output=error.output.decode(), key_id=key_id, key_server=key_server
            )

    @classmethod
    def _find_asset_with_key_id(
        cls, *, key_id: str, keys_path: Path
    ) -> Tuple[str, Optional[Path]]:
        # First look for any key asset that matches the key_id fingerprint.
        for key_path in keys_path.glob(pattern="*.asc"):
            key = key_path.read_text()
            if key_id in cls._get_key_fingerprints(key=key):
                return key_id, key_path

        # Handle case where user uses 'key_id' as the name of the key asset.
        # In this case we translate the key name to a proper key ID.
        key_path = keys_path / f"{key_id}.asc"
        if key_path.exists():
            fingerprints = cls._get_key_fingerprints(key=key_path.read_text())
            if len(fingerprints) == 0:
                logger.warning(f"Error reading key file: {key_path}")
                return key_id, None
            elif len(fingerprints) > 1:
                logger.warning(f"Found multiple key fingerprints in: {key_path}")
            return fingerprints[0], key_path

        return key_id, None

    @classmethod
    def install_gpg_key_id(
        cls, *, key_id: str, keys_path: Path, key_server: Optional[str] = None
    ) -> bool:
        # If key_id references a local asset, we search and replace the local
        # key_id reference with the actual fingerprint suitable for checking
        # if it is installed.
        key_id, key_path = cls._find_asset_with_key_id(
            key_id=key_id, keys_path=keys_path
        )

        # If key is already installed, nothing to do.
        if cls._is_key_id_installed(key_id):
            return False

        # Install key if it is available as a local asset.
        if key_path is not None:
            cls._install_gpg_key(key_id=key_id, key=key_path.read_text())
            return True

        # Finally attempt to install from keyserver.
        cls._install_gpg_key_id_from_keyserver(key_id=key_id, key_server=key_server)
        logger.debug(f"Installed apt repository key ID: {key_id}")
        return True

    @classmethod
    def _get_ppa_parts(cls, ppa: str) -> Tuple[str, str]:
        ppa_split = ppa.split("/")
        if len(ppa_split) != 2:
            raise errors.AptPPAInstallError(ppa=ppa, reason="invalid PPA format")
        return ppa_split[0], ppa_split[1]

    @classmethod
    def _get_launchpad_ppa_key_id(cls, ppa: str) -> str:
        owner, name = cls._get_ppa_parts(ppa)
        launchpad = Launchpad.login_anonymously("snapcraft", "production")
        launchpad_url = f"~{owner}/+archive/{name}"

        logger.debug(f"Loading launchpad url: {launchpad_url}")
        try:
            key_id = launchpad.load(launchpad_url).signing_key_fingerprint
        except lazr.restfulclient.errors.NotFound as error:
            raise errors.AptPPAInstallError(
                ppa=ppa, reason="not found on launchpad"
            ) from error

        logger.debug(f"Retrieved launchpad PPA key ID: {key_id}")
        return key_id

    @classmethod
    def install_ppa(cls, *, keys_path: Path, ppa: str) -> bool:
        owner, name = cls._get_ppa_parts(ppa)
        key_id = cls._get_launchpad_ppa_key_id(ppa)

        return any(
            [
                cls.install_gpg_key_id(keys_path=keys_path, key_id=key_id),
                cls.install_sources(
                    components=["main"],
                    deb_types=["deb"],
                    name=f"ppa-{owner}_{name}",
                    suites=["$SNAPCRAFT_APT_RELEASE"],
                    url=f"http://ppa.launchpad.net/{owner}/{name}/ubuntu",
                ),
            ]
        )

    @classmethod
    def _construct_deb822_source(
        cls,
        *,
        architectures: Optional[List[str]] = None,
        components: List[str],
        deb_types: Optional[List[str]] = None,
        suites: List[str],
        url: str,
    ) -> str:
        with io.StringIO() as deb822:
            if deb_types:
                deb_text = " ".join(deb_types)
                print(f"Types: {deb_text}", file=deb822)

            url_text = _format_sources_list(url)
            print(f"URIs: {url_text}", file=deb822)

            suites_text = _format_sources_list(" ".join(suites))
            print(f"Suites: {suites_text}", file=deb822)

            components_text = " ".join(components)
            print(f"Components: {components_text}", file=deb822)

            if architectures:
                arch_text = " ".join(architectures)
                host_arch = _get_host_arch()
                arch_text = arch_text.replace("$SNAPCRAFT_APT_HOST_ARCH", host_arch)
                print(f"Architectures: {arch_text}", file=deb822)

            return deb822.getvalue()

    @classmethod
    def install_sources(
        cls,
        *,
        architectures: Optional[List[str]] = None,
        components: List[str],
        deb_types: Optional[List[str]] = None,
        name: str,
        suites: List[str],
        url: str,
    ) -> bool:
        config = cls._construct_deb822_source(
            architectures=architectures,
            components=components,
            deb_types=deb_types,
            suites=suites,
            url=url,
        )

        if name not in ["default", "default-security"]:
            name = "snapcraft-" + name

        config_path = Path(f"/etc/apt/sources.list.d/{name}.sources")
        if config_path.exists() and config_path.read_text() == config:
            # Already installed and matches, nothing to do.
            logger.debug(f"Ignoring unchanged sources: {config_path}")
            return False

        _sudo_write_file(dst_path=config_path, content=config.encode())
        logger.debug(f"Installed sources: {config_path}")
        return True

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


def _format_sources_list(sources_list: str):
    release = os_release.OsRelease().version_codename()

    return sources_list.replace("$SNAPCRAFT_APT_RELEASE", release)
