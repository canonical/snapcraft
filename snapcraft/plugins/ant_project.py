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
import snapcraft.common
import sys


class AntProjectPlugin(snapcraft.BasePlugin):

    def pull(self):
        return self.handle_source_options()

    def build(self):
        if not self.run(['ant']):
            return False
        files = glob.glob(os.path.join(self.builddir, 'target', '*.jar'))
        if not files:
            snapcraft.common.log('Could not find any built jar files for part %s' % self.name)
            sys.exit(1)
        jardir = os.path.join(self.installdir, 'jar')
        return self.run(['mkdir', '-p', jardir]) and \
            self.run(['cp', '-a'] + files + [jardir])

    def env(self, root):
        jars = glob.glob(os.path.join(self.installdir, 'jar', '*.jar'))
        if not jars:
            return []
        jars = [os.path.join(root, 'jar', os.path.basename(x)) for x in jars]
        return ['CLASSPATH=%s:$CLASSPATH' % ':'.join(jars)]
