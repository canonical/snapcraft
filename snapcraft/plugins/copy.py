# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017 Canonical Ltd
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

This plugin is DEPRECATED in favor of the `dump` plugin.

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

import snapcraft
from snapcraft.internal import errors

logger = logging.getLogger(__name__)


class CopyPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["files"] = {
            "type": "object",
            "additionalProperties": {"type": "string", "minLength": 1},
        }

        # The `files` keyword is required here, but the `source` keyword is
        # not. It should default to the current working directory.
        schema["required"].append("files")

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return super().get_build_properties() + ["files"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        logger.warning(
            "DEPRECATED: The 'copy' plugin's functionality "
            "has been replaced by the 'dump' plugin, and it will "
            "soon be removed."
        )

    def enable_cross_compilation(self):
        pass

    def build(self):
        super().build()

        files = self.options.files
        globs = {f: files[f] for f in files if glob.has_magic(f)}
        filepaths = {
            os.path.join(self.builddir, f): files[f]
            for f in files
            if not glob.has_magic(f)
        }

        for src in globs:
            paths = glob.glob(os.path.join(self.builddir, src))
            if not paths:
                raise errors.SnapcraftEnvironmentError(
                    "no matches for {!r}".format(src)
                )
            for path in paths:
                filepaths.update({os.path.join(self.builddir, path): globs[src]})

        for src in sorted(filepaths):
            dst = os.path.join(self.installdir, filepaths[src].lstrip("/"))
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            _recursively_link(src, dst, self.installdir)


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
            # Only follow symlinks that are NOT pointing at libc (LP: #1658774)
            if link not in snapcraft.repo.Repo.get_package_libraries("libc6"):
                follow_symlinks = True

    snapcraft.common.link_or_copy(source, destination, follow_symlinks=follow_symlinks)


def _recursively_link(source, destination, boundary):
    if os.path.isdir(source):
        if os.path.isdir(destination):
            destination = os.path.join(destination, os.path.basename(source))
        elif os.path.exists(destination):
            raise NotADirectoryError(
                "Cannot overwrite non-directory {!r} with directory "
                "{!r}".format(destination, source)
            )
        snapcraft.file_utils.link_or_copy_tree(
            source,
            destination,
            copy_function=lambda src, dst: _link_or_copy(src, dst, boundary),
        )
    else:
        _link_or_copy(source, destination, boundary)
