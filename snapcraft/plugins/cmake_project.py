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

import snapcraft


class CMakePlugin(snapcraft.BasePlugin):
    def __init__(self, name, config, options):
        super().__init__(name, config, options)
        if self.options.configflags is None:
            self.options.configflags = ''

    def build(self):
        return self.run('cmake . -DCMAKE_INSTALL_PREFIX= %s' % self.options.configflags) and \
            self.run("make") and \
            self.run("make install DESTDIR=" + self.installdir)
