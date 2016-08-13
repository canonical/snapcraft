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

"""This plugin just dumps the content from a specified source.

This plugin uses the common plugin keywords as well as those for 'sources'.
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

In the cases where dumping the content needs some mangling or organizing
one would take advantage of the core functionalities available to plugins
such as: `filesets`, `stage`, `snap` and `organize`.
"""

import os
import shutil

import snapcraft


class DumpPlugin(snapcraft.BasePlugin):

    def build(self):
        super().build()
        _link_tree(self.builddir, self.installdir, self.installdir)


def _link_tree(source_tree, destination_tree, boundary):
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

    _create_similar_directory(source_tree, destination_tree)

    for root, directories, files in os.walk(source_tree):
        for directory in directories:
            source = os.path.join(root, directory)
            destination = os.path.join(
                destination_tree, os.path.relpath(source, source_tree))

            _create_similar_directory(source, destination)

        for file_name in files:
            source = os.path.join(root, file_name)
            destination = os.path.join(
                destination_tree, os.path.relpath(source, source_tree))

            _link_or_copy(source, destination, boundary)


def _create_similar_directory(source, destination):
    os.makedirs(destination, exist_ok=True)
    shutil.copystat(source, destination, follow_symlinks=False)


def _link_or_copy(source, destination, boundary):
    """Attempt to copy symlinks as symlinks unless pointing out of boundary."""

    follow_symlinks = False

    # If this is a symlink, analyze where it's pointing and make sure it will
    # still be valid when snapped. If it won't, follow the symlink when
    # copying (i.e. copy the file to which the symlink is pointing instead).
    if os.path.islink(source):
        link = os.readlink(source)
        destination_dirname = os.path.dirname(destination)
        normalized = os.path.normpath(os.path.join(destination_dirname, link))
        if os.path.isabs(link) or not normalized.startswith(boundary):
            follow_symlinks = True

    try:
        snapcraft.common.link_or_copy(source, destination,
                                      follow_symlinks=follow_symlinks)
    except FileNotFoundError:
        raise FileNotFoundError(
            '{!r} is a broken symlink pointing outside the snap'.format(
                source))
