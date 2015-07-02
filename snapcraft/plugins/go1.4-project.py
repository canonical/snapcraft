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
import sys


class Go14ProjectPlugin(snapcraft.BasePlugin):
    def __init__(self, name, options):
        super().__init__(name, options)
        if self.options.source.startswith("lp:"):
            self.fullname = self.options.source.split(":~")[1]
        else:
            self.fullname = self.options.source.split("://")[1]
        self.godir = os.path.join(os.path.join(os.getcwd(), "parts", self.name))
        try:
            os.makedirs(self.godir)
        except FileExistsError:
            pass

    def env(self):
        return [
            "GOPATH=%s" % self.godir,
        ]

    def pull(self):
        return self.run("go get -t %s" % (self.fullname), self.godir)

    def build(self):
        return self.run("go build %s" % (self.fullname), self.godir)

    def stage(self):
        if not self.run("go install %s" % (self.fullname), self.godir):
            return False
        return self.run("cp -rf %s %s" % (
            os.path.join(self.godir, "bin"), self.stagedir))

    def snap(self):
        return self.doDeploy(["bin"])
