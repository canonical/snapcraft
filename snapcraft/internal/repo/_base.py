# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
import fileinput
import glob
import itertools
import logging
import os
import re
import shutil
import stat

from typing import List, Optional, Set

from snapcraft import file_utils
from snapcraft.internal import mangling, xattrs
from . import errors


logger = logging.getLogger(__name__)


class BaseRepo:
    """Base implementation for a platform specific repo handler.

    Generally any new repo handling system would inherit from this and
    implement:

    - get
    - unpack
    - get_package_libraries
    - get_packages_for_source_type
    - refresh_build_packages
    - install_build_packages
    - is_package_installed

    At the end of the `unpack` method `normalize` needs to be called to
    adapt the artifacts downloaded to be generic enough for building a snap."""

    @classmethod
    def get_package_libraries(cls, package_name: str) -> Set[str]:
        """Return a list of libraries in package_name.

        Given the contents of package_name, return the subset of what are
        considered libraries from those contents, be it static or shared.

        :param str package: package name to get library contents from.
        :returns: a list of libraries that package_name provides. This includes
                  directories.
        :rtype: set.
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def get_package_for_file(cls, file_path: str) -> str:
        """Return the package name that provides file_path.

        :param str file_path: the absolute path to the file to search for.
        :returns: package name that provides file_path.
        :rtype: str
        :raises snapcraft.repo.errors.FileProviderNotFound:
            if file_path is not provided by any package.
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def get_packages_for_source_type(cls, source_type: str) -> Set[str]:
        """Return a list of packages required to to work with source_type.

        source_type can be any of the following:

        - bzr
        - deb
        - rpm
        - git
        - hg
        - mercurial
        - subversion
        - svn
        - tar
        - zip

        :param str source_type: a VCS source type to handle.
        :returns: a set of packages that need to be installed on the host.
        :rtype: set of strings.
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def refresh_build_packages(cls) -> None:
        """Refresh the build packages cache.

        If refreshing is not possible
        snapcraft.repo.errors.CacheUpdateFailedError should be raised

        :raises snapcraft.repo.errors.NoNativeBackendError:
            if the method is not implemented in the subclass.
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def install_build_packages(cls, package_names: List[str]) -> List[str]:
        """Install packages on the host required to build.

        This method needs to be implemented by using the appropriate method
        to install packages on the system. If possible they should be marked
        as automatically installed to allow for easy removal.
        The method should return a list of the actually installed packages
        in the form "package=version".

        If one of the packages cannot be found
        snapcraft.repo.errors.BuildPackageNotFoundError should be raised.
        If dependencies for a package cannot be resolved
        snapcraft.repo.errors.PackageBrokenError should be raised.
        If installing a package on the host failed
        snapcraft.repo.errors.BuildPackagesNotInstalledError should be raised.

        :param package_names: a list of package names to install.
        :type package_names: a list of strings.
        :return: a list with the packages installed and their versions.
        :rtype: list of strings.
        :raises snapcraft.repo.errors.NoNativeBackendError:
            if the method is not implemented in the subclass.
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def build_package_is_valid(cls, package_name: str) -> bool:
        """Check that a given package is valid on the host.

        :param package_name: a package name to check.
        :type package_name: str
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def is_package_installed(cls, package_name: str) -> bool:
        """Return a bool indicating if package_name is installed.

        :param str package_name: the package name to query.
        :returns: True if package_name is installed if not False.
        :rtype: boolean
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def get_installed_packages(cls) -> List[str]:
        """Return a list of the installed packages and their versions

        :rtype: list of strings with the form package=version.
        """
        raise errors.NoNativeBackendError()

    @classmethod
    def install_stage_packages(
        cls, *, package_names: List[str], install_dir: str
    ) -> List[str]:
        """Install stage packages to install_dir."""
        raise errors.NoNativeBackendError()

    @classmethod
    def install_gpg_key(cls, *, key_id: str, key: str) -> bool:
        """Install trusted GPG key."""
        raise errors.NoNativeBackendError()

    @classmethod
    def install_source(cls, *, name: str, source: str) -> bool:
        """Install repository source location."""
        raise errors.NoNativeBackendError()

    @classmethod
    def normalize(cls, unpackdir: str) -> None:
        """Normalize artifacts in unpackdir.

        Repo specific packages are generally created to live in a specific
        distro. What normalize does is scan through the unpacked artifacts
        and slightly modifies them to work better with snapcraft projects
        when building and to also work within a snap's environment.

        :param str unpackdir: directory where files where unpacked.
        """
        cls._remove_useless_files(unpackdir)
        cls._fix_artifacts(unpackdir)
        cls._fix_xml_tools(unpackdir)
        cls._fix_shebangs(unpackdir)

    @classmethod
    def _mark_origin_stage_package(
        cls, sources_dir: str, stage_package: str
    ) -> Set[str]:
        """Mark all files in sources_dir as coming from stage_package."""
        file_list = set()
        for (root, dirs, files) in os.walk(sources_dir):
            for file_name in files:
                file_path = os.path.join(root, file_name)

                # Mark source.
                xattrs.write_origin_stage_package(file_path, stage_package)

                file_path = os.path.relpath(root, sources_dir)
                file_list.add(file_path)

        return file_list

    @classmethod
    def _remove_useless_files(cls, unpackdir: str) -> None:
        """Remove files that aren't useful or will clash with other parts."""
        sitecustomize_files = glob.glob(
            os.path.join(unpackdir, "usr", "lib", "python*", "sitecustomize.py")
        )
        for sitecustomize_file in sitecustomize_files:
            os.remove(sitecustomize_file)

    @classmethod
    def _fix_artifacts(cls, unpackdir: str) -> None:
        """Perform various modifications to unpacked artifacts.

        Sometimes distro packages will contain absolute symlinks (e.g. if the
        relative path would go all the way to root, they just do absolute). We
        can't have that, so instead clean those absolute symlinks.

        Some unpacked items will also contain suid binaries which we do not
        want in the resulting snap.
        """
        for root, dirs, files in os.walk(unpackdir):
            # Symlinks to directories will be in dirs, while symlinks to
            # non-directories will be in files.
            for entry in itertools.chain(files, dirs):
                path = os.path.join(root, entry)
                if os.path.islink(path) and os.path.isabs(os.readlink(path)):
                    cls._fix_symlink(path, unpackdir, root)
                elif os.path.exists(path):
                    _fix_filemode(path)

                if path.endswith(".pc") and not os.path.islink(path):
                    fix_pkg_config(unpackdir, path)

    @classmethod
    def _fix_xml_tools(cls, unpackdir: str) -> None:
        xml2_config_path = os.path.join(unpackdir, "usr", "bin", "xml2-config")
        with contextlib.suppress(FileNotFoundError):
            file_utils.search_and_replace_contents(
                xml2_config_path,
                re.compile(r"prefix=/usr"),
                "prefix={}/usr".format(unpackdir),
            )

        xslt_config_path = os.path.join(unpackdir, "usr", "bin", "xslt-config")
        with contextlib.suppress(FileNotFoundError):
            file_utils.search_and_replace_contents(
                xslt_config_path,
                re.compile(r"prefix=/usr"),
                "prefix={}/usr".format(unpackdir),
            )

    @classmethod
    def _fix_symlink(cls, path: str, unpackdir: str, root: str) -> None:
        host_target = os.readlink(path)
        if host_target in cls.get_package_libraries("libc6"):
            logger.debug(
                "Not fixing symlink {!r}: it's pointing to libc".format(host_target)
            )
            return

        target = os.path.join(unpackdir, os.readlink(path)[1:])
        if not os.path.exists(target) and not _try_copy_local(path, target):
            return
        os.remove(path)
        os.symlink(os.path.relpath(target, root), path)

    @classmethod
    def _fix_shebangs(cls, unpackdir: str) -> None:
        """Change hard-coded shebangs in unpacked files to use env."""
        mangling.rewrite_python_shebangs(unpackdir)


class DummyRepo(BaseRepo):
    @classmethod
    def get_packages_for_source_type(cls, source_type: str) -> Set[str]:
        return set()


def _try_copy_local(path: str, target: str) -> bool:
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


def fix_pkg_config(
    root: str, pkg_config_file: str, prefix_trim: Optional[str] = None
) -> None:
    """Opens a pkg_config_file and prefixes the prefix with root."""
    pattern_trim = None
    if prefix_trim:
        pattern_trim = re.compile("^prefix={}(?P<prefix>.*)".format(prefix_trim))
    pattern = re.compile("^prefix=(?P<prefix>.*)")

    with fileinput.input(pkg_config_file, inplace=True) as input_file:
        for line in input_file:
            match = pattern.search(line)
            if prefix_trim is not None and pattern_trim is not None:
                match_trim = pattern_trim.search(line)
            if prefix_trim is not None and match_trim is not None:
                print("prefix={}{}".format(root, match_trim.group("prefix")))
            elif match:
                print("prefix={}{}".format(root, match.group("prefix")))
            else:
                print(line, end="")


def _fix_filemode(path: str) -> None:
    mode = stat.S_IMODE(os.stat(path, follow_symlinks=False).st_mode)
    if mode & 0o4000 or mode & 0o2000:
        logger.warning("Removing suid/guid from {}".format(path))
        os.chmod(path, mode & 0o1777)
