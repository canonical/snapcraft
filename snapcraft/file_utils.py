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

import os
import shutil
import logging


logger = logging.getLogger(__name__)


def replace_in_file(directory, file_pattern, search_pattern, replacement):
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
                _search_and_replace_contents(os.path.join(root, file_name),
                                             search_pattern, replacement)


def link_or_copy(source, destination, follow_symlinks=False):
    """Hard-link source and destination files. Copy if it fails to link.

    Hard-linking may fail (e.g. a cross-device link, or permission denied), so
    as a backup plan we just copy it.

    :param str source: The source to which destination will be linked.
    :param str destination: The destination to be linked to source.
    :param bool follow_symlinks: Whether or not symlinks should be followed.
    """

    try:
        # Note that follow_symlinks doesn't seem to work for os.link, so we'll
        # implement this logic ourselves using realpath.
        source_path = source
        if follow_symlinks:
            source_path = os.path.realpath(source)

        if not os.path.exists(os.path.dirname(destination)):
            create_similar_directory(
                os.path.dirname(source_path),
                os.path.dirname(destination))
        # Setting follow_symlinks=False in case this bug is ever fixed
        # upstream-- we want this function to continue supporting NOT following
        # symlinks.
        os.link(source_path, destination, follow_symlinks=False)
    except OSError:
        shutil.copy2(source, destination, follow_symlinks=follow_symlinks)
        uid = os.stat(source, follow_symlinks=follow_symlinks).st_uid
        gid = os.stat(source, follow_symlinks=follow_symlinks).st_gid
        try:
            os.chown(destination, uid, gid, follow_symlinks=follow_symlinks)
        except PermissionError as e:
            logger.debug('Unable to chown {destination}: {error}'.format(
                destination=destination, error=e))


def link_or_copy_tree(source_tree, destination_tree,
                      copy_function=link_or_copy):
    """Copy a source tree into a destination, hard-linking if possile.

    :param str source_tree: Source directory to be copied.
    :param str destination_tree: Destination directory. If this directory
                                 already exists, the files in `source_tree`
                                 will take precedence.
    :param str boundary: Filesystem boundary no symlinks are allowed to cross.
    """

    if not os.path.isdir(source_tree):
        raise NotADirectoryError('{!r} is not a directory'.format(source_tree))

    if (not os.path.isdir(destination_tree) and
            os.path.exists(destination_tree)):
        raise NotADirectoryError(
            'Cannot overwrite non-directory {!r} with directory '
            '{!r}'.format(destination_tree, source_tree))

    create_similar_directory(source_tree, destination_tree)

    for root, directories, files in os.walk(source_tree):
        for directory in directories:
            source = os.path.join(root, directory)
            destination = os.path.join(
                destination_tree, os.path.relpath(source, source_tree))

            create_similar_directory(source, destination)

        for file_name in files:
            source = os.path.join(root, file_name)
            destination = os.path.join(
                destination_tree, os.path.relpath(source, source_tree))

            copy_function(source, destination)


def create_similar_directory(source, destination, follow_symlinks=False):
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
        logger.debug('Unable to chown {}: {}'.format(destination, exception))

    shutil.copystat(source, destination, follow_symlinks=follow_symlinks)


def _search_and_replace_contents(file_path, search_pattern, replacement):
    # Don't bother trying to rewrite a symlink. It's either invalid or the
    # linked file will be rewritten on its own.
    if os.path.islink(file_path):
        return

    with open(file_path, 'r+') as f:
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
