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

import glob
import os
import snapcraft

class MavenPlugin(snapcraft.BasePlugin):

    def pull(self):
        return self.pull_branch(self.options.source)

    def build(self):
        if not self.run(['mvn', 'package']):
            return False
        for g in ['*.jar', '*.war']:
            files = glob.glob(os.path.join(self.builddir, 'target', g))
            if not files:
                continue
            if not self.run(['cp', '-a'] + files + [self.installdir]):
                return False
        return True

