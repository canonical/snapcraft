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

import snapcraft


logger = logging.getLogger(__name__)


class CopyPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['files'] = {
                'type': 'object',
            }
        schema['required'].append('files')
        return schema

    def build(self):
        # Sources handling first
        super().build()

        for src in sorted(self.options.files):
            dst = self.options.files[src]
            if not os.path.lexists(src):
                # If it doesn't exist, check if it is a glob
                filelst = glob.glob(os.path.join(self.sourcedir, src))
                if True in map(os.path.lexists, filelst):
                    # Ensure the destination is a directory
                    if dst[-1] is not '/':
                        dst += '/'
                else:
                    raise EnvironmentError('file "{}" missing'.format(src))

            # Expand directories to include part info
            dst = os.path.join(self.installdir, dst)
            src = os.path.join(self.sourcedir, src)

            # Ensure we have a destination to copy to
            os.makedirs(os.path.dirname(dst), exist_ok=True)

            # DO IT! DO IT NOW!
            self.run(['cp', '--preserve=all', '-R', src, dst])
