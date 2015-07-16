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

import apt
import os
import snapcraft.common
import subprocess
import sys

from snapcraft.plugins.ubuntu import UbuntuPlugin


class QmlPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        options.package = None
        super().__init__(name, options)

        class QmlPackageOptions:
            package = ["qmlscene", "qtdeclarative5-qtmir-plugin", "mir-graphics-drivers-desktop", "qtubuntu-desktop"]
            recommends = False

        self.ubuntu = UbuntuPlugin(name, QmlPackageOptions())

    def pull(self):
        return self.ubuntu.pull()

    def snap_files(self):
        return self.ubuntu.snap_files()

    def build(self):
        return self.ubuntu.build()


