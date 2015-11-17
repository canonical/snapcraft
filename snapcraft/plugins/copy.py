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
        for src in sorted(self.options.files):
            dst = self.options.files[src]
            if not os.path.lexists(src):
                raise EnvironmentError('file "{}" missing'.format(src))
            dst = os.path.join(self.installdir, dst)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            self.run(['cp', '--preserve=all', '-R', src, dst],
                     cwd=os.getcwd())
