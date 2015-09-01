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


class Python2Plugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        super().__init__(name, options)

        class UbuntuOptions:
            packages = ['python-dev']

        ubuntu_options = UbuntuOptions()

        if options.requirements:
            self.requirements = options.requirements
            ubuntu_options.packages.append('python-pip')
        else:
            self.requirements = None

        self.ubuntu = ubuntu.UbuntuPlugin(name, ubuntu_options)

    # note that we don't need to set PYTHONHOME here,
    # python discovers this automatically from it installed
    # location.  And PATH is automatically set by snapcraft.

    def pull(self):
        if not self.ubuntu.pull():
            return False
        if self.requirements and not self.run(
                ['python2', '-m', 'pip', 'install', '-r', os.path.join(os.getcwd(), self.requirements)]):
            return False
        return True

    def build(self):
        return self.ubuntu.build()

    def snap_files(self):
        return self.ubuntu.snap_files()
