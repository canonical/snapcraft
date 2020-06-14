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
import gnupg
import io
import lazr.restfulclient.errors
import logging
import os
import pathlib
import re
import subprocess
import sys
import tempfile
from typing import Dict, List, Optional, Set, Tuple  # noqa: F401
from typing_extensions import Final

from launchpadlib.launchpad import Launchpad
from xdg import BaseDirectory

from snapcraft import file_utils
from snapcraft.project._project_options import ProjectOptions
from snapcraft.internal import os_release
from snapcraft.internal.indicators import is_dumb_terminal

from . import errors
from .apt_cache import AptCache
from ._base import BaseRepo, get_pkg_name_parts


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


def _get_dpkg_list_path(base: str) -> pathlib.Path:
    return pathlib.Path(f"/snap/{base}/current/usr/share/snappy/dpkg.list")


def get_packages_in_base(*, base: str) -> List[str]:
    # We do not want to break what we already have.
    if base in ("core", "core16", "core18"):
        return _DEFAULT_FILTERED_STAGE_PACKAGES

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
            package_list.append(line.split()[1])

    # format of package_list is <package_name>[:<architecture>]
    return package_list


def _sudo_write_file(*, dst_path: pathlib.Path, content: bytes) -> None:
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
                installed_version = apt_cache.get_installed_version(pkg_name)

                if installed_version is None or (
                    pkg_version is not None and installed_version != pkg_version
                ):
                    return False

        return True

    @classmethod
    def _get_marked_packages(cls, package_names: List[str]) -> List[Tuple[str, str]]:
        with AptCache() as apt_cache:
            try:
                apt_cache.mark_packages(set(package_names))
            except errors.PackageNotFoundError as error:
                raise errors.BuildPackageNotFoundError(error.package_name)

            return apt_cache.get_marked_packages()

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

        if cls._check_if_all_packages_installed(package_names):
            # To get the version list required to return, we mark the installed
            # packages, but no refresh is required.
            marked_packages = cls._get_marked_packages(package_names)
        else:
            # Ensure we have an up-to-date cache first.
            cls.refresh_build_packages()

            marked_packages = cls._get_marked_packages(package_names)

            # TODO: this matches prior behavior, but the version is not
            # being passed along to apt-get install, even if prescribed
            # by the user.  We should specify it upon user request.
            install_packages = sorted([name for name, _ in marked_packages])

            cls._install_packages(install_packages)

        return sorted([f"{name}={version}" for name, version in marked_packages])

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

    @classmethod
    def fetch_stage_packages(
        cls, *, package_names: List[str], base: str, stage_packages_path: pathlib.Path
    ) -> List[str]:
        logger.debug(f"Requested stage-packages: {sorted(package_names)!r}")

        installed: Set[str] = set()

        stage_packages_path.mkdir(exist_ok=True)
        with AptCache(stage_cache=_STAGE_CACHE_DIR) as apt_cache:
            filter_packages = set(get_packages_in_base(base=base))
            apt_cache.update()
            apt_cache.mark_packages(set(package_names))
            apt_cache.unmark_packages(
                required_names=set(package_names), filtered_names=filter_packages
            )
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
        for pkg_path in stage_packages_path.glob("*.deb"):
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
        cls, *, key_id: str, keys_path: pathlib.Path
    ) -> Tuple[str, Optional[pathlib.Path]]:
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
        cls, *, key_id: str, keys_path: pathlib.Path, key_server: Optional[str] = None
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
    def install_ppa(cls, *, keys_path: pathlib.Path, ppa: str) -> bool:
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

        config_path = pathlib.Path(f"/etc/apt/sources.list.d/{name}.sources")
        if config_path.exists() and config_path.read_text() == config:
            # Already installed and matches, nothing to do.
            logger.debug(f"Ignoring unchanged sources: {config_path}")
            return False

        _sudo_write_file(dst_path=config_path, content=config.encode())
        logger.debug(f"Installed sources: {config_path}")
        return True

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


def _format_sources_list(sources_list: str):
    release = os_release.OsRelease().version_codename()

    return sources_list.replace("$SNAPCRAFT_APT_RELEASE", release)
