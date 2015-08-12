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
from snapcraft.plugins import ubuntu


class Python2ProjectPlugin(snapcraft.BasePlugin):

    # note that we don't need to setup env(), python figures it out
    # see python2.py for more details

    def __init__(self, name, options):
        super().__init__(name, options)

        if options.requirements:
            self.requirements = options.requirements

            class UbuntuOptions:
                packages = ["python-pip"]
            self.ubuntu = ubuntu.UbuntuPlugin(name, UbuntuOptions())
        else:
            self.requirements = None

    def pull(self):
        if not self.ubuntu.pull():
            return False
        if self.requirements and not self.run(
                ['python2', '-m', 'pip', 'install', '-r', self.requirements, "--prefix={}/usr".format(self.installdir)]):
            return False
        return self.handle_source_options()

    def build(self):
        return self.run(
            ["python2", "setup.py", "install", "--install-layout=deb",
             "--prefix={}/usr".format(self.installdir)])
