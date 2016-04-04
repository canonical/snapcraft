# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

"""The copy plugin is useful for assets or other sources with no build system.

This plugin uses the common plugin keywords as well as those for 'sources'
(though the 'source' keyword is optional). For more information check the
'plugins' topic for the former and the 'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - files:
      (object)
      A dictionary of key-value pairs. The key is the current location of the
      file relative to snapcraft.yaml (unless `source` is specified, in which
      case it's relative to the root of the source). The value is where to
      place the file in-snap, and is relative to the root of the snap. This
      works like `cp -r <key> <value>`. Note that globbing is supported for the
      key, allowing one to use *, ?, and character ranges expressed with [].
"""

import logging
import os
import glob
import shutil

import snapcraft


logger = logging.getLogger(__name__)


class CopyPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties']['files'] = {
            'type': 'object',
        }

        # The `files` keyword is required here, but the `source` keyword is
        # not. It should default to the current working directory.
        schema['required'].append('files')
        schema['required'].remove('source')
        schema['properties']['source']['default'] = '.'

        return schema

    def build(self):
        super().build()

        files = self.options.files
        globs = {f: files[f] for f in files if glob.has_magic(f)}
        filepaths = {os.path.join(self.builddir, f): files[f] for f in files
                     if not glob.has_magic(f)}

        for src in globs:
            paths = glob.glob(os.path.join(self.builddir, src))
            if not paths:
                raise EnvironmentError('no matches for {!r}'.format(src))
            for path in paths:
                filepaths.update(
                    {os.path.join(self.builddir, path): globs[src]})

        for src in sorted(filepaths):
            dst = os.path.join(self.installdir, filepaths[src].lstrip('/'))
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            _recursively_link(src, dst)


def _recursively_link(source, destination):
    if os.path.isdir(source):
        if os.path.isdir(destination):
            destination = os.path.join(destination, os.path.basename(source))
        elif os.path.exists(destination):
            raise NotADirectoryError(
                'Cannot overwrite non-directory {!r} with directory '
                '{!r}'.format(destination, source))
        _linktree(source, destination)
    else:
        snapcraft.common.link_or_copy(source, destination,
                                      follow_symlinks=False)


def _create_similar_directory(source, destination):
    os.makedirs(destination, exist_ok=True)
    shutil.copystat(source, destination, follow_symlinks=False)


def _linktree(source_tree, destination_tree):
    if not os.path.isdir(destination_tree):
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

            snapcraft.common.link_or_copy(
                source, destination, follow_symlinks=False)
