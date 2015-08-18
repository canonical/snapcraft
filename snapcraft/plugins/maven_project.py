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
import sys

import snapcraft


class MavenPlugin(snapcraft.BasePlugin):

    def pull(self):
        return self.handle_source_options()

    def build(self):
        if not self.run(['mvn', 'package']):
            return False
        jarfiles = glob.glob(os.path.join(self.builddir, 'target', '*.jar'))
        warfiles = glob.glob(os.path.join(self.builddir, 'target', '*.war'))
        if not (jarfiles or warfiles):
            snapcraft.common.log('Could not find any built jar or war files for part %s' % self.name)
            sys.exit(1)
        if jarfiles:
            jardir = os.path.join(self.installdir, 'jar')
            if not os.makedirs(jardir, exist_ok=True):
                return False
            if not self.run(['cp', '-a'] + jarfiles + [jardir]):
                return False
        if warfiles:
            wardir = os.path.join(self.installdir, 'war')
            if not os.makedirs(wardir, exist_ok=True):
                return False
            if not self.run(['cp', '-a'] + warfiles + [wardir]):
                return False
        return True
