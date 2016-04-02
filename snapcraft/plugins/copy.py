# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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


"""The copy plugin enable local or remote content copy to the stage directory.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter. If source is omitted, it's implicitely set
to '.'

Additionally, this plugin uses the following plugin-specific keywords:

    - files:
      (list of strings)
      'src: dest' format of files or directories to copy to
    - destination:
      (string)
      the destination to copy whole source content

The 2 keywords are mutually exclusive. If none is set, destination is set
to "."
"""

from contextlib import suppress
import logging
import os
import glob
import shutil

import snapcraft
from snapcraft import common
from snapcraft import sources


logger = logging.getLogger(__name__)


_ADDITIONAL_FILES_TO_IGNORE = ['.git*', '.bzr', '.svn', '*.snap']


class CopyPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties'].update({
            'destination': {
                'type': 'string',
            },
            'files': {
                'type': 'object',
            },
        })
        schema.pop('required')
        return schema

    def __init__(self, name, options):
        super().__init__(name, options)
        self.options.destination = getattr(options, 'destination', None)
        self.options.files = getattr(options, 'files', None)
        self.options.source = getattr(options, 'source', None)
        if self.options.files:
            if self.options.destination:
                raise ValueError('You can either specify a destination or '
                                 'files options, not both')
        else:
            # set default to "." for argument
            if not self.options.destination:
                self.options.destination = "."
            if os.path.isabs(self.options.destination):
                raise ValueError('path "{}" must be relative'
                                 .format(self.options.destination))
        # set default to "." for argument
        if not self.options.source:
            self.options.source = "."

    def _get_ignore_file_list(self, src, names):
        """List files to ignore when doing a copy"""
        # files to ignore are in root directory
        if os.path.realpath(src) != os.getcwd():
            return []
        files_to_ignores = _ADDITIONAL_FILES_TO_IGNORE.copy()
        files_to_ignores.extend(common.SNAPCRAFT_FILES)
        ignored_file_list = []
        for expr in files_to_ignores:
            ignored_file_list.extend(glob.glob(expr))
        return ignored_file_list

    def build(self):
        # do a full copy, no filtering
        if not self.options.files:
            installdir = os.path.realpath(
                os.path.join(self.installdir, self.options.destination))
            # tar file to extract
            if sources.get_source_type(self.options) == "tar":
                os.makedirs(installdir, exist_ok=True)
                self.tar = snapcraft.sources.Tar(self.options.source,
                                                 self.sourcedir)
                print(installdir)
                self.tar.provision(installdir)
                print(installdir)
            # other content (branch or local)
            else:
                with suppress(FileNotFoundError):
                    os.rmdir(installdir)
                shutil.copytree(self.sourcedir, installdir,
                                ignore=self._get_ignore_file_list)

        # file filtering through files option
        else:
            files = self.options.files
            globs = {os.path.join(self.sourcedir, f): files[f]
                     for f in files if glob.has_magic(f)}
            filepaths = {os.path.join(self.sourcedir, f): files[f]
                         for f in files if not glob.has_magic(f)}

            for src in globs:
                globbed = glob.glob(src)
                if not globbed:
                    raise EnvironmentError('no matches for {!r}'.format(src))
                for g in globbed:
                    filepaths.update({g: globs[src]})

            for src in sorted(filepaths):
                dst = os.path.join(self.installdir, filepaths[src])
                os.makedirs(os.path.dirname(dst), exist_ok=True)

                if os.path.isdir(src):
                    shutil.copytree(src, dst)
                else:
                    shutil.copy2(src, dst)
