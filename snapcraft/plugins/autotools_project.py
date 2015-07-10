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


class AutotoolsPlugin(snapcraft.BasePlugin):
    def __init__(self, name, options):
        super().__init__(name, options)
        if self.options.configflags is None:
            self.options.configflags = ''

    def pull(self):
        return self.pullBranch(self.options.source)

    def build(self):
        if not os.path.exists(os.path.join(self.builddir, "configure")):
            if not self.run("env NOCONFIGURE=1 ./autogen.sh"):
                return False
        if not self.run("./configure --prefix= " + self.options.configflags):
            return False
        return self.run("make all") and self.run("make install DESTDIR=" + self.installdir)

    def snapFiles(self):
        return (['*'], ['include'])
