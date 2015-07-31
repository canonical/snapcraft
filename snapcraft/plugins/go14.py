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

import os

import snapcraft
from snapcraft.plugins import ubuntu


class GoPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        super().__init__(name, options)

        class UbuntuOptions:
            package = 'golang-go'
        self.ubuntu = ubuntu.UbuntuPlugin(name, UbuntuOptions())

        # Prefix our files so they don't conflict with anything else,
        # especially since we aren't even installing them into the snap.
        # TODO: This can go away if/when we have proper dependency flattening.
        self.ubuntu.installdir = os.path.join(self.installdir, 'go')

    def pull(self):
        return self.ubuntu.pull()

    def build(self):
        return self.ubuntu.build()

    def env(self, root):
        return self.ubuntu.env(root) + [
            "PATH=%s/go/usr/lib/go/bin:$PATH" % root,
            "GOROOT=%s/go/usr/lib/go" % root,
        ]

    def snap_files(self):
        return ([], [])  # Go statically links everything, no need for runtime
