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

import logging
import os
import glob
import shutil

import snapcraft


logger = logging.getLogger(__name__)


class CopyPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        return {
            'properties': {
                'files': {
                    'type': 'object',
                },
            },
            'required': [
                'files',
            ]
        }

    def build(self):
        files = self.options.files
        globs = {f: files[f] for f in files if glob.has_magic(f)}
        filepaths = {f: files[f] for f in files if not glob.has_magic(f)}

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
