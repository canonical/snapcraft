# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import contextlib
import glob
import hashlib
import logging
import os
import re
import shutil
import stat
import string
import subprocess
import sys
import urllib
import urllib.request
from typing import Dict, Set, List, Tuple  # noqa: F401

import apt
from xml.etree import ElementTree

import snapcraft
from snapcraft import file_utils
from snapcraft.internal import cache, repo, common, os_release
from snapcraft.internal.indicators import is_dumb_terminal
from ._base import BaseRepo
from . import errors


logger = logging.getLogger(__name__)

_DEFAULT_SOURCES = """deb http://${prefix}.ubuntu.com/${suffix}/ ${release} main restricted
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates main restricted
deb http://${prefix}.ubuntu.com/${suffix}/ ${release} universe
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates universe
deb http://${prefix}.ubuntu.com/${suffix}/ ${release} multiverse
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates multiverse
deb http://${security}.ubuntu.com/${suffix} ${release}-security main restricted
deb http://${security}.ubuntu.com/${suffix} ${release}-security universe
deb http://${security}.ubuntu.com/${suffix} ${release}-security multiverse
"""
_GEOIP_SERVER = "http://geoip.ubuntu.com/lookup"
_library_list = dict()  # type: Dict[str, Set[str]]
_HASHSUM_MISMATCH_PATTERN = re.compile(r"(E:Failed to fetch.+Hash Sum mismatch)+")


class _AptCache:
    def __init__(self, deb_arch, *, sources_list=None, use_geoip=False):
        self._deb_arch = deb_arch
        self._sources_list = sources_list
        self._use_geoip = use_geoip

    def _setup_apt(self, cache_dir):
        # Do not install recommends
        apt.apt_pkg.config.set("Apt::Install-Recommends", "False")

        # Methods and solvers dir for when in the SNAP
        if common.is_snap():
            snap_dir = os.getenv("SNAP")
            apt_dir = os.path.join(snap_dir, "usr", "lib", "apt")
            apt.apt_pkg.config.set("Dir", apt_dir)
            # yes apt is broken like that we need to append os.path.sep
            methods_dir = os.path.join(apt_dir, "methods")
            apt.apt_pkg.config.set("Dir::Bin::methods", methods_dir + os.path.sep)
            solvers_dir = os.path.join(apt_dir, "solvers")
            apt.apt_pkg.config.set("Dir::Bin::solvers::", solvers_dir + os.path.sep)
            apt_key_path = os.path.join(snap_dir, "usr", "bin", "apt-key")
            apt.apt_pkg.config.set("Dir::Bin::apt-key", apt_key_path)
            gpgv_path = os.path.join(snap_dir, "usr", "bin", "gpgv")
            apt.apt_pkg.config.set("Apt::Key::gpgvcommand", gpgv_path)
            apt.apt_pkg.config.set("Dir::Etc::Trusted", "/etc/apt/trusted.gpg")
            apt.apt_pkg.config.set("Dir::Etc::TrustedParts", "/etc/apt/trusted.gpg.d/")

        # Make sure we always use the system GPG configuration, even with
        # apt.Cache(rootdir).
        for key in "Dir::Etc::Trusted", "Dir::Etc::TrustedParts":
            apt.apt_pkg.config.set(key, apt.apt_pkg.config.find_file(key))

        # Clear up apt's Post-Invoke-Success as we are not running
        # on the system.
        apt.apt_pkg.config.clear("APT::Update::Post-Invoke-Success")

        self.progress = apt.progress.text.AcquireProgress()
        if is_dumb_terminal():
            # Make output more suitable for logging.
            self.progress.pulse = lambda owner: True
            self.progress._width = 0

        sources_list_file = os.path.join(cache_dir, "etc", "apt", "sources.list")

        os.makedirs(os.path.dirname(sources_list_file), exist_ok=True)
        with open(sources_list_file, "w") as f:
            f.write(self._collected_sources_list())

        # dpkg also needs to be in the rootdir in order to support multiarch
        # (apt calls dpkg --print-foreign-architectures).
        dpkg_path = shutil.which("dpkg")
        if dpkg_path:
            # Symlink it into place
            destination = os.path.join(cache_dir, dpkg_path[1:])
            if not os.path.exists(destination):
                os.makedirs(os.path.dirname(destination), exist_ok=True)
                os.symlink(dpkg_path, destination)
        else:
            logger.warning("Cannot find 'dpkg' command needed to support multiarch")

        return self._create_cache(cache_dir, sources_list_file)

    def _create_cache(self, cache_dir: str, sources_list_file: str) -> apt.Cache:
        apt_cache = apt.Cache(rootdir=cache_dir, memonly=True)
        try:
            apt_cache.update(
                fetch_progress=self.progress, sources_list=sources_list_file
            )
        except apt.cache.FetchFailedException as e:
            # In case of a hashsum mismatch, clear the index and try again.
            # Re-raise the error if that failed.
            if re.match(_HASHSUM_MISMATCH_PATTERN, str(e)):
                try:
                    index = apt.apt_pkg.config.find_dir("Dir::State::Lists")
                    shutil.rmtree(index)
                    apt_cache = apt.Cache(rootdir=cache_dir, memonly=True)
                    apt_cache.update(
                        fetch_progress=self.progress, sources_list=sources_list_file
                    )
                except apt.cache.FetchFailedException as retry:
                    raise errors.CacheUpdateFailedError(str(retry))
            else:
                raise errors.CacheUpdateFailedError(str(e))
        return apt_cache

    @contextlib.contextmanager
    def archive(self, cache_dir):
        try:
            apt_cache = self._setup_apt(cache_dir)
            apt_cache.open()

            try:
                yield apt_cache
            finally:
                apt_cache.close()
        except Exception as e:
            logger.debug("Exception occurred: {!r}".format(e))
            raise e

    def sources_digest(self):
        return hashlib.sha384(
            self._collected_sources_list().encode(sys.getfilesystemencoding())
        ).hexdigest()

    def _collected_sources_list(self):
        if self._use_geoip or self._sources_list:
            release = os_release.OsRelease()
            return _format_sources_list(
                self._sources_list,
                deb_arch=self._deb_arch,
                use_geoip=self._use_geoip,
                release=release.version_codename(),
            )

        return _get_local_sources_list()

    def fetch_binary(self, *, package_candidate, destination: str) -> None:
        # This is a workaround for the overly verbose python-apt we use.
        # There is an unreleased patch which once released could replace
        # this code https://salsa.debian.org/apt-team/python-apt/commit/d122f9142df614dbb5f7644112280140dc155ecc  # noqa
        # What follows is almost a tit for tat implementation of upstream's
        # fetch_binary logic.
        base = os.path.basename(package_candidate._records.filename)
        destfile = os.path.join(destination, base)
        if apt.package._file_is_same(
            destfile, package_candidate.size, package_candidate._records.md5_hash
        ):
            logging.debug("Ignoring already existing file: {}".format(destfile))
            return os.path.abspath(destfile)
        acq = apt.apt_pkg.Acquire(self.progress)
        acqfile = apt.apt_pkg.AcquireFile(
            acq,
            package_candidate.uri,
            package_candidate._records.md5_hash,
            package_candidate.size,
            base,
            destfile=destfile,
        )
        acq.run()

        if acqfile.status != acqfile.STAT_DONE:
            raise apt.package.FetchError(
                "The item %r could not be fetched: %s"
                % (acqfile.destfile, acqfile.error_text)
            )

        return os.path.abspath(destfile)


class Ubuntu(BaseRepo):
    @classmethod
    def get_package_libraries(cls, package_name):
        global _library_list
        if package_name not in _library_list:
            output = (
                subprocess.check_output(["dpkg", "-L", package_name])
                .decode(sys.getfilesystemencoding())
                .strip()
                .split()
            )
            _library_list[package_name] = {
                i for i in output if ("lib" in i and os.path.isfile(i))
            }

        return _library_list[package_name].copy()

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
        new_packages = []  # type: List[Tuple[str, str]]
        with apt.Cache() as apt_cache:
            try:
                cls._mark_install(apt_cache, package_names)
            except errors.PackageNotFoundError as e:
                raise errors.BuildPackageNotFoundError(e.package_name)
            for package in apt_cache.get_changes():
                new_packages.append((package.name, package.candidate.version))

        if new_packages:
            cls._install_new_build_packages([package[0] for package in new_packages])
        return ["{}={}".format(package[0], package[1]) for package in new_packages]

    @classmethod
    def _mark_install(cls, apt_cache: apt.Cache, package_names: List[str]) -> None:
        for name in package_names:
            if name.endswith(":any"):
                name = name[:-4]
            if apt_cache.is_virtual_package(name):
                name = apt_cache.get_providing_packages(name)[0].name
            logger.debug(
                "Marking {!r} (and its dependencies) to be fetched".format(name)
            )
            name_arch, version = repo.get_pkg_name_parts(name)
            try:
                if version:
                    _set_pkg_version(apt_cache[name_arch], version)
                # Disable automatic resolving of broken packages here
                # because if that fails it raises a SystemError and the
                # API doesn't expose enough information about he problem.
                # Instead we let apt-get show a verbose error message later.
                # Also, make sure this package is marked as auto-installed,
                # which will propagate to its dependencies.
                apt_cache[name_arch].mark_install(auto_fix=False, from_user=False)

                # Now mark this package as NOT automatically installed, which
                # will leave its dependencies marked as auto-installed, which
                # allows us to clean them up if necessary.
                apt_cache[name_arch].mark_auto(False)

                cls._verify_marked_install(apt_cache[name_arch])
            except KeyError:
                raise errors.PackageNotFoundError(name)

    @classmethod
    def _verify_marked_install(cls, package: apt.Package):
        if not package.installed and not package.marked_install:
            broken_deps = []  # type: List[str]
            for deps in package.candidate.dependencies:
                for dep in deps:
                    if not dep.target_versions:
                        broken_deps.append(dep.name)
            raise errors.PackageBrokenError(package.name, broken_deps)

    @classmethod
    def _install_new_build_packages(cls, package_names: List[str]) -> None:
        package_names.sort()
        logger.info("Installing build dependencies: %s", " ".join(package_names))
        env = os.environ.copy()
        env.update(
            {"DEBIAN_FRONTEND": "noninteractive", "DEBCONF_NONINTERACTIVE_SEEN": "true"}
        )

        apt_command = ["sudo", "apt-get", "--no-install-recommends", "-y"]
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
    def build_package_is_valid(cls, package_name):
        with apt.Cache() as apt_cache:
            return package_name in apt_cache

    @classmethod
    def is_package_installed(cls, package_name):
        with apt.Cache() as apt_cache:
            return apt_cache[package_name].installed

    @classmethod
    def get_installed_packages(cls):
        installed_packages = []
        with apt.Cache() as apt_cache:
            for package in apt_cache:
                if package.installed:
                    installed_packages.append(
                        "{}={}".format(package.name, package.installed.version)
                    )
        return installed_packages

    def __init__(self, rootdir, sources=None, project_options=None) -> None:
        super().__init__(rootdir)
        self._downloaddir = os.path.join(rootdir, "download")

        if not project_options:
            project_options = snapcraft.ProjectOptions()

        self._apt = _AptCache(
            project_options.deb_arch,
            sources_list=sources,
            use_geoip=project_options.use_geoip,
        )

        self._cache = cache.AptStagePackageCache(
            sources_digest=self._apt.sources_digest()
        )

    def is_valid(self, package_name):
        with self._apt.archive(self._cache.base_dir) as apt_cache:
            return package_name in apt_cache

    def get(self, package_names) -> None:
        with self._apt.archive(self._cache.base_dir) as apt_cache:
            self._mark_install(apt_cache, package_names)
            self._filter_base_packages(apt_cache, package_names)
            self._autokeep_packages(apt_cache)
            return self._get(apt_cache)

    def _autokeep_packages(self, apt_cache):
        for package in apt_cache.get_changes():
            if package.is_auto_removable:
                package.mark_keep()

    def _filter_base_packages(self, apt_cache, package_names):
        manifest_dep_names = self._manifest_dep_names(apt_cache)

        skipped_essential = []
        skipped_blacklisted = []

        # unmark some base packages here
        # note that this will break the consistency check inside apt_cache
        # (apt_cache.broken_count will be > 0)
        # but that is ok as it was consistent before we excluded
        # these base package
        for pkg in apt_cache:
            # those should be already on each system, it also prevents
            # diving into downloading libc6
            if pkg.candidate.priority in "essential" and pkg.name not in package_names:
                skipped_essential.append(pkg.name)
                pkg.mark_keep()
                continue
            if pkg.name in manifest_dep_names and pkg.name not in package_names:
                skipped_blacklisted.append(pkg.name)
                pkg.mark_keep()
                continue

        if skipped_essential:
            logger.debug(
                "Skipping priority essential packages: "
                "{!r}".format(skipped_essential)
            )
        if skipped_blacklisted:
            logger.debug(
                "Skipping blacklisted from manifest packages: "
                "{!r}".format(skipped_blacklisted)
            )

    def _get(self, apt_cache):
        # Ideally we'd use apt.Cache().fetch_archives() here, but it seems to
        # mangle some package names on disk such that we can't match it up to
        # the archive later. We could get around this a few different ways:
        #
        # 1. Store each stage package in the cache named by a hash instead of
        #    its name from the archive.
        # 2. Download packages in a different manner.
        #
        # In the end, (2) was chosen for minimal overhead and a simpler cache
        # implementation. So we're using fetch_binary() here instead.
        # Downloading each package individually has the drawback of witholding
        # any clue of how long the whole pulling process will take, but that's
        # something we'll have to live with.
        pkg_list = []
        for package in apt_cache.get_changes():
            pkg_list.append(str(package.candidate))
            source = self._apt.fetch_binary(
                package_candidate=package.candidate,
                destination=self._cache.packages_dir,
            )
            destination = os.path.join(self._downloaddir, os.path.basename(source))
            with contextlib.suppress(FileNotFoundError):
                os.remove(destination)
            file_utils.link_or_copy(source, destination)

        return pkg_list

    def unpack(self, unpackdir) -> None:
        pkgs_abs_path = glob.glob(os.path.join(self._downloaddir, "*.deb"))
        for pkg in pkgs_abs_path:
            # TODO needs elegance and error control
            try:
                subprocess.check_call(["dpkg-deb", "--extract", pkg, unpackdir])
            except subprocess.CalledProcessError:
                raise errors.UnpackError(pkg)
        self.normalize(unpackdir)

    def _manifest_dep_names(self, apt_cache):
        manifest_dep_names = set()

        with open(os.path.abspath(os.path.join(__file__, "..", "manifest.txt"))) as f:
            for line in f:
                pkg = line.strip()
                if pkg in apt_cache:
                    manifest_dep_names.add(pkg)

        return manifest_dep_names


def _get_local_sources_list():
    sources_list = glob.glob("/etc/apt/sources.list.d/*.list")
    sources_list.append("/etc/apt/sources.list")

    sources = ""
    for source in sources_list:
        with open(source) as f:
            sources += f.read()

    return sources


def _get_geoip_country_code_prefix():
    try:
        with urllib.request.urlopen(_GEOIP_SERVER) as f:
            xml_data = f.read()
        et = ElementTree.fromstring(xml_data)
        cc = et.find("CountryCode")
        if cc is None:
            return ""
        return cc.text.lower()
    except (ElementTree.ParseError, urllib.error.URLError):
        pass
    return ""


def _format_sources_list(sources_list, *, deb_arch, use_geoip=False, release="xenial"):
    if not sources_list:
        sources_list = _DEFAULT_SOURCES

    if deb_arch in ("amd64", "i386"):
        if use_geoip:
            geoip_prefix = _get_geoip_country_code_prefix()
            prefix = "{}.archive".format(geoip_prefix)
        else:
            prefix = "archive"
        suffix = "ubuntu"
        security = "security"
    else:
        prefix = "ports"
        suffix = "ubuntu-ports"
        security = "ports"

    return string.Template(sources_list).substitute(
        {"prefix": prefix, "release": release, "suffix": suffix, "security": security}
    )


def _fix_filemode(path):
    mode = stat.S_IMODE(os.stat(path, follow_symlinks=False).st_mode)
    if mode & 0o4000 or mode & 0o2000:
        logger.warning("Removing suid/guid from {}".format(path))
        os.chmod(path, mode & 0o1777)


def _try_copy_local(path, target):
    real_path = os.path.realpath(path)
    if os.path.exists(real_path):
        logger.warning(
            "Copying needed target link from the system {}".format(real_path)
        )
        os.makedirs(os.path.dirname(target), exist_ok=True)
        shutil.copyfile(os.readlink(path), target)
        return True
    else:
        logger.warning("{} will be a dangling symlink".format(path))
        return False


def _set_pkg_version(pkg, version):
    """Set cadidate version to a specific version if available"""
    if version in pkg.versions:
        version = pkg.versions.get(version)
        pkg.candidate = version
    else:
        raise errors.PackageNotFoundError("{}={}".format(pkg.name, version))
