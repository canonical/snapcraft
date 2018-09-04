# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

from contextlib import contextmanager, suppress
import errno
import hashlib
import logging
import re
import os
import shutil
import subprocess
import sys
from typing import Pattern, Callable, Generator, List
from typing import Set  # noqa F401

from snapcraft.internal import common
from snapcraft.internal.errors import (
    RequiredCommandFailure,
    RequiredCommandNotFound,
    RequiredPathDoesNotExist,
    SnapcraftEnvironmentError,
    SnapcraftCopyFileNotFoundError,
    ToolMissingError,
)

if sys.version_info < (3, 6):
    import sha3  # noqa


logger = logging.getLogger(__name__)


def replace_in_file(
    directory: str, file_pattern: Pattern, search_pattern: Pattern, replacement: str
) -> None:
    """Searches and replaces patterns that match a file pattern.

    :param str directory: The directory to look for files.
    :param str file_pattern: The file pattern to match inside directory.
    :param search_pattern: A re.compile'd pattern to search for within
                           matching files.
    :param str replacement: The string to replace the matching search_pattern
                            with.
    """

    for root, directories, files in os.walk(directory):
        for file_name in files:
            if file_pattern.match(file_name):
                file_path = os.path.join(root, file_name)
                # Don't bother trying to rewrite a symlink. It's either invalid
                # or the linked file will be rewritten on its own.
                if not os.path.islink(file_path):
                    search_and_replace_contents(file_path, search_pattern, replacement)


def search_and_replace_contents(
    file_path: str, search_pattern: Pattern, replacement: str
) -> None:
    """Search file and replace any occurrence of pattern with replacement.

    :param str file_path: Path of file to be searched.
    :param re.RegexObject search_pattern: Pattern for which to search.
    :param str replacement: The string to replace pattern.
    """
    try:
        with open(file_path, "r+") as f:
            try:
                original = f.read()
            except UnicodeDecodeError:
                # This was probably a binary file. Skip it.
                return

            replaced = search_pattern.sub(replacement, original)
            if replaced != original:
                f.seek(0)
                f.truncate()
                f.write(replaced)
    except PermissionError as e:
        logger.warning(
            "Unable to open {path} for writing: {error}".format(path=file_path, error=e)
        )


def link_or_copy(source: str, destination: str, follow_symlinks: bool = False) -> None:
    """Hard-link source and destination files. Copy if it fails to link.

    Hard-linking may fail (e.g. a cross-device link, or permission denied), so
    as a backup plan we just copy it.

    :param str source: The source to which destination will be linked.
    :param str destination: The destination to be linked to source.
    :param bool follow_symlinks: Whether or not symlinks should be followed.
    """

    try:
        link(source, destination, follow_symlinks=follow_symlinks)
    except OSError as e:
        if e.errno == errno.EEXIST and not os.path.isdir(destination):
            # os.link will fail if the destination already exists, so let's
            # remove it and try again.
            os.remove(destination)
            link_or_copy(source, destination, follow_symlinks)
        else:
            copy(source, destination, follow_symlinks=follow_symlinks)


def link(source: str, destination: str, *, follow_symlinks: bool = False) -> None:
    """Hard-link source and destination files.

    :param str source: The source to which destination will be linked.
    :param str destination: The destination to be linked to source.
    :param bool follow_symlinks: Whether or not symlinks should be followed.

    :raises SnapcraftCopyFileNotFoundError: If source doesn't exist.
    """
    # Note that follow_symlinks doesn't seem to work for os.link, so we'll
    # implement this logic ourselves using realpath.
    source_path = source
    if follow_symlinks:
        source_path = os.path.realpath(source)

    if not os.path.exists(os.path.dirname(destination)):
        create_similar_directory(
            os.path.dirname(source_path), os.path.dirname(destination)
        )
    # Setting follow_symlinks=False in case this bug is ever fixed
    # upstream-- we want this function to continue supporting NOT following
    # symlinks.
    try:
        os.link(source_path, destination, follow_symlinks=False)
    except FileNotFoundError:
        raise SnapcraftCopyFileNotFoundError(source)


def copy(source: str, destination: str, *, follow_symlinks: bool = False) -> None:
    """Copy source and destination files.

    This function overwrites the destination if it already exists, and also
    tries to copy ownership information.

    :param str source: The source to be copied to destination.
    :param str destination: Where to put the copy.
    :param bool follow_symlinks: Whether or not symlinks should be followed.

    :raises SnapcraftCopyFileNotFoundError: If source doesn't exist.
    """
    # If os.link raised an I/O error, it may have left a file behind. Skip on
    # OSError in case it doesn't exist or is a directory.
    with suppress(OSError):
        os.unlink(destination)

    try:
        shutil.copy2(source, destination, follow_symlinks=follow_symlinks)
    except FileNotFoundError:
        raise SnapcraftCopyFileNotFoundError(source)
    uid = os.stat(source, follow_symlinks=follow_symlinks).st_uid
    gid = os.stat(source, follow_symlinks=follow_symlinks).st_gid
    try:
        os.chown(destination, uid, gid, follow_symlinks=follow_symlinks)
    except PermissionError as e:
        logger.debug(
            "Unable to chown {destination}: {error}".format(
                destination=destination, error=e
            )
        )


def link_or_copy_tree(
    source_tree: str,
    destination_tree: str,
    ignore: Callable[[str, List[str]], List[str]] = None,
    copy_function: Callable[..., None] = link_or_copy,
) -> None:
    """Copy a source tree into a destination, hard-linking if possible.

    :param str source_tree: Source directory to be copied.
    :param str destination_tree: Destination directory. If this directory
                                 already exists, the files in `source_tree`
                                 will take precedence.
    :param callable ignore: If given, called with two params, source dir and
                            dir contents, for every dir copied. Should return
                            list of contents to NOT copy.
    :param callable copy_function: Callable that actually copies.
    """

    if not os.path.isdir(source_tree):
        raise NotADirectoryError("{!r} is not a directory".format(source_tree))

    if not os.path.isdir(destination_tree) and (
        os.path.exists(destination_tree) or os.path.islink(destination_tree)
    ):
        raise NotADirectoryError(
            "Cannot overwrite non-directory {!r} with directory "
            "{!r}".format(destination_tree, source_tree)
        )

    create_similar_directory(source_tree, destination_tree)

    destination_basename = os.path.basename(destination_tree)

    for root, directories, files in os.walk(source_tree, topdown=True):
        ignored = set()  # type: Set[str]
        if ignore is not None:
            ignored = set(ignore(root, directories + files))

        # Don't recurse into destination tree if it's a subdirectory of the
        # source tree.
        if os.path.relpath(destination_tree, root) == destination_basename:
            ignored.add(destination_basename)

        if ignored:
            # Prune our search appropriately given an ignore list, i.e. don't
            # walk into directories that are ignored.
            directories[:] = [d for d in directories if d not in ignored]

        for directory in directories:
            source = os.path.join(root, directory)
            # os.walk doesn't by default follow symlinks (which is good), but
            # it includes symlinks that are pointing to directories in the
            # directories list. We want to treat it as a file, here.
            if os.path.islink(source):
                files.append(directory)
                continue

            destination = os.path.join(
                destination_tree, os.path.relpath(source, source_tree)
            )

            create_similar_directory(source, destination)

        for file_name in set(files) - ignored:
            source = os.path.join(root, file_name)
            destination = os.path.join(
                destination_tree, os.path.relpath(source, source_tree)
            )

            copy_function(source, destination)


def create_similar_directory(
    source: str, destination: str, follow_symlinks: bool = False
) -> None:
    """Create a directory with the same permission bits and owner information.

    :param str source: Directory from which to copy name, permission bits, and
                       owner information.
    :param str destintion: Directory to create and to which the `source`
                           information will be copied.
    :param bool follow_symlinks: Whether or not symlinks should be followed.
    """

    stat = os.stat(source, follow_symlinks=follow_symlinks)
    uid = stat.st_uid
    gid = stat.st_gid
    os.makedirs(destination, exist_ok=True)
    try:
        os.chown(destination, uid, gid, follow_symlinks=follow_symlinks)
    except PermissionError as exception:
        logger.debug("Unable to chown {}: {}".format(destination, exception))

    shutil.copystat(source, destination, follow_symlinks=follow_symlinks)


def executable_exists(path: str) -> bool:
    """Return True if 'path' exists and is readable and executable."""
    return os.path.exists(path) and os.access(path, os.R_OK | os.X_OK)


@contextmanager
def requires_command_success(
    command: str, not_found_fmt: str = None, failure_fmt: str = None
) -> Generator:
    if isinstance(command, str):
        cmd_list = command.split()
    else:
        raise TypeError("command must be a string.")
    kwargs = dict(command=command, cmd_list=cmd_list)
    try:
        subprocess.check_call(cmd_list, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        if not_found_fmt is not None:
            kwargs["fmt"] = not_found_fmt
        raise RequiredCommandNotFound(**kwargs)
    except subprocess.CalledProcessError:
        if failure_fmt is not None:
            kwargs["fmt"] = failure_fmt
        raise RequiredCommandFailure(**kwargs)
    yield


@contextmanager
def requires_path_exists(path: str, error_fmt: str = None) -> Generator:
    if not os.path.exists(path):
        kwargs = dict(path=path)
        if error_fmt is not None:
            kwargs["fmt"] = error_fmt
        raise RequiredPathDoesNotExist(**kwargs)
    yield


def calculate_sha3_384(path: str) -> str:
    """Calculate sha3 384 hash, reading the file in 1MB chunks."""
    return calculate_hash(path, algorithm="sha3_384")


def calculate_hash(path: str, *, algorithm: str) -> str:
    """Calculate the hash for path with algorithm."""
    # This will raise an AttributeError if algorithm is unsupported
    hasher = getattr(hashlib, algorithm)()

    blocksize = 2 ** 20
    with open(path, "rb") as f:
        while True:
            buf = f.read(blocksize)
            if not buf:
                break
            hasher.update(buf)
    return hasher.hexdigest()


def get_tool_path(command_name: str) -> str:
    """Return the path to the given command

    Return a path to command_name, if Snapcraft is running out of the snap
    or in legacy mode (snap or sources), it ensures it is using the one in
    the snap, not the host.
    If a path cannot be resolved, ToolMissingError is raised.

    : param str command_name: the name of the command to resolve a path for.
    :raises ToolMissingError: if command_name cannot be resolved to a path.
    :return: Path to command
    :rtype: str
    """
    if common.is_snap():
        command_path = _command_path_in_root(os.getenv("SNAP"), command_name)
    else:
        command_path = shutil.which(command_name)

    # shutil.which will return None if it cannot find command_name but
    # _command_path_in_root will return an empty string.
    if not command_path:
        raise ToolMissingError(command_name=command_name)

    return command_path


def _command_path_in_root(root, command_name: str) -> str:
    for bin_directory in (
        os.path.join("usr", "local", "sbin"),
        os.path.join("usr", "local", "bin"),
        os.path.join("usr", "sbin"),
        os.path.join("usr", "bin"),
        os.path.join("sbin"),
        os.path.join("bin"),
    ):
        path = os.path.join(root, bin_directory, command_name)
        if os.path.exists(path):
            return path

    return ""


def get_linker_version_from_file(linker_file: str) -> str:
    """Returns the version of the linker from linker_file.

    linker_file must be of the format ld-(?P<linker_version>[\d.]+).so$

    :param str linker_file: a valid file path or basename representing
                            the linker from libc6 or related.
    :returns: the version extracted from the linker file.
    :rtype: string
    :raises snapcraft.internal.errors.SnapcraftEnvironmentError:
       if linker_file is not of the expected format.
    """
    m = re.search(r"ld-(?P<linker_version>[\d.]+).so$", linker_file)
    if not m:
        # This is a programmatic error, we don't want to be friendly
        # about this.
        raise SnapcraftEnvironmentError(
            "The format for the linker should be of the of the form "
            "<root>/ld-<X>.<Y>.so. {!r} does not match that format. "
            "Ensure you are targeting an appropriate base".format(linker_file)
        )
    linker_version = m.group("linker_version")

    return linker_version
