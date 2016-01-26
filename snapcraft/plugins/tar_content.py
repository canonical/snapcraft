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

import os
import snapcraft
import snapcraft.sources


class TarContentPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        return {
            'properties': {
                'source': {
                    'type': 'string',
                },
                'destination': {
                    'type': 'string',
                },
            },
            'required': [
                'source',
            ]
        }

    def __init__(self, name, options):
        super().__init__(name, options)
        if not self.options.destination:
            builddir = self.builddir
        else:
            if os.path.isabs(self.options.destination):
                raise ValueError('path "{}" must be relative'
                                 .format(self.options.destination))

            builddir = os.path.join(self.builddir, self.options.destination)
            if not os.path.exists(builddir):
                os.makedirs(builddir)

        self.tar = snapcraft.sources.Tar(self.options.source, builddir)

    def pull(self):
        self.tar.pull()

    def build(self):
        if not self.options.destination:
            installdir = self.installdir
        else:
            installdir = os.path.join(self.installdir,
                                      self.options.destination)
            if not os.path.exists(installdir):
                os.makedirs(installdir)

        self.tar.provision(installdir)
