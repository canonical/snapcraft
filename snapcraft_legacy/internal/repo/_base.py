# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2022 Canonical Ltd
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
import pathlib
import re
import shutil
import stat
from typing import List, Optional, Set

from snapcraft_legacy import file_utils
from snapcraft_legacy.internal import mangling, xattrs

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
        :raises snapcraft_legacy.repo.errors.FileProviderNotFound:
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
        snapcraft_legacy.repo.errors.CacheUpdateFailedError should be raised

        :raises snapcraft_legacy.repo.errors.NoNativeBackendError:
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
        snapcraft_legacy.repo.errors.BuildPackageNotFoundError should be raised.
        If dependencies for a package cannot be resolved
        snapcraft_legacy.repo.errors.PackageBrokenError should be raised.
        If installing a package on the host failed
        snapcraft_legacy.repo.errors.BuildPackagesNotInstalledError should be raised.

        :param package_names: a list of package names to install.
        :type package_names: a list of strings.
        :return: a list with the packages installed and their versions.
        :rtype: list of strings.
        :raises snapcraft_legacy.repo.errors.NoNativeBackendError:
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
    def fetch_stage_packages(
        cls,
        *,
        package_names: List[str],
        base: str,
        stage_packages_path: pathlib.Path,
        target_arch: str,
    ) -> List[str]:
        """Fetch stage packages to stage_packages_path."""
        raise errors.NoNativeBackendError()

    @classmethod
    def unpack_stage_packages(
        cls, *, stage_packages_path: pathlib.Path, install_path: pathlib.Path
    ) -> None:
        """Unpack stage packages to install_path."""
        raise errors.NoNativeBackendError()

    @classmethod
    def install_gpg_key(cls, *, key_id: str, key: str) -> bool:
        """Install trusted GPG key."""
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
                if os.path.islink(path):
                    cls._fix_symlink(path, unpackdir)
                elif os.path.exists(path):
                    _fix_filemode(path)

                if (
                    path.endswith(".pc")
                    and os.path.isfile(path)
                    and not os.path.islink(path)
                ):
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
    def _fix_symlink(cls, symlink_path: str, unpack_dir: str) -> None:
        """Verify, fix, and/or log symlinks that would break in snap."""
        symlink_target = os.readlink(symlink_path)
        resolved_path = os.path.realpath(symlink_path)

        # If symlink doesn't point to absolute target, nothing to do.
        if not os.path.isabs(symlink_target):
            return

        # Ignore anything linking to libc6.
        if resolved_path in cls.get_package_libraries("libc6"):
            logger.debug("Not fixing symlink %r: it's pointing to libc", symlink_target)
            return

        # Use relative symlink if target is found in unpack directory.
        target_in_unpack_dir = os.path.join(unpack_dir, symlink_target[1:])
        relative_target = os.path.relpath(
            target_in_unpack_dir, os.path.dirname(symlink_path)
        )
        # Copy the file from host if we need it, but warn about it.
        if not os.path.exists(target_in_unpack_dir) and os.path.isfile(resolved_path):
            logger.warning(
                "Copying needed symlink target %r from host to satisfy %r.",
                resolved_path,
                symlink_path,
            )
            os.makedirs(os.path.dirname(target_in_unpack_dir), exist_ok=True)
            shutil.copyfile(resolved_path, target_in_unpack_dir)

        # Relink to relative path.
        if os.path.exists(target_in_unpack_dir):
            os.unlink(symlink_path)
            os.symlink(relative_target, symlink_path)
            return

        # Cannot find target, issue a warning.
        relative_link = os.path.relpath(symlink_path, unpack_dir)
        logger.warning("%r will be a dangling symlink", relative_link)

    @classmethod
    def _fix_shebangs(cls, unpackdir: str) -> None:
        """Change hard-coded shebangs in unpacked files to use env."""
        mangling.rewrite_python_shebangs(unpackdir)


class DummyRepo(BaseRepo):
    @classmethod
    def get_packages_for_source_type(cls, source_type: str) -> Set[str]:
        return set()


def fix_pkg_config(
    prefix_prepend: str, pkg_config_file: str, prefix_trim: Optional[str] = None
) -> None:
    """Fix the prefix parameter in pkg-config files.

    This function does 3 things:
    1. Remove `prefix_trim` from the prefix.
    2. Remove directories commonly added by staged snaps from the prefix.
    3. Prepend `prefix_prepend` to the prefix.

    The prepended stage directory depends on the source of the pkg-config file:
    - From snaps built via launchpad: `/build/<snap-name>/stage`
    - From snaps built via a provider: `/root/stage`
    - From snaps built locally: `<local-path-to-project>/stage`
    - Built during the build stage: the install directory

    :param pkg_config_file: pkg-config (.pc) file to modify
    :param prefix_prepend: directory to prepend to the prefix
    :param prefix_trim: directory to remove from prefix
    """
    # build patterns
    prefixes_to_trim = [r"/build/[\w\-. ]+/stage", "/root/stage"]
    if prefix_trim:
        prefixes_to_trim.append(prefix_trim)
    pattern_trim = re.compile(
        f"^prefix=(?P<trim>{'|'.join(prefixes_to_trim)})(?P<prefix>.*)"
    )
    pattern = re.compile("^prefix=(?P<prefix>.*)")
    pattern_pcfiledir = re.compile("^prefix *= *[$]{pcfiledir}.*")

    # process .pc file
    with fileinput.input(pkg_config_file, inplace=True) as input_file:
        for line in input_file:
            if pattern_pcfiledir.search(line) is not None:
                print(line, end="")
                continue
            match = pattern.search(line)
            match_trim = pattern_trim.search(line)

            if match_trim is not None:
                # trim prefix and prepend new data
                new_prefix = f"prefix={prefix_prepend}{match_trim.group('prefix')}"
            elif match:
                # nothing to trim, so only prepend new data
                new_prefix = f"prefix={prefix_prepend}{match.group('prefix')}"

            if match_trim is not None or match:
                print(new_prefix)
                logger.debug(
                    f"For pkg-config file {pkg_config_file}, prefix was changed from"
                    f" {line} to {new_prefix}"
                )
            else:
                print(line, end="")


def _fix_filemode(path: str) -> None:
    mode = stat.S_IMODE(os.stat(path, follow_symlinks=False).st_mode)
    if mode & 0o4000 or mode & 0o2000:
        logger.warning("Removing suid/guid from {}".format(path))
        os.chmod(path, mode & 0o1777)


def get_pkg_name_parts(pkg_name):
    """Break package name into base parts"""

    name = pkg_name
    version = None
    with contextlib.suppress(ValueError):
        name, version = pkg_name.split("=")

    return name, version
