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

# FIXME: add support for i386
URLS = {
    'amd64': "https://storage.googleapis.com/golang/go1.4.2.linux-amd64.tar.gz",
}


class Go14Plugin(snapcraft.BasePlugin):
    def __init__(self, name, options):
        super().__init__(name, options)
        # FIXME: horrible
        self.arch = "amd64"
        self.godir = os.path.join(
            os.path.join(os.getcwd(), "parts", self.name))
        self.goroot = os.path.join(os.path.join(
            os.getcwd(), "parts", "go1.4", "go"))
        self.gorootbin = os.path.join(self.goroot, "bin")
        self.tar_file = os.path.join(
            self.godir, os.path.basename(URLS[self.arch]))
        self.makedirs(self.godir)

    def env(self):
        return [
            "PATH=%s:$PATH" % self.gorootbin,
            "GOROOT=%s" % self.goroot,
        ]

    def pull(self):
        # FIXME: use the internal downloader (once its there) to get stuff
        if not self.run("wget -c %s " % URLS[self.arch], cwd=self.godir):
            return False
        if not os.path.exists(os.path.join(self.godir, "go/bin/go")):
            return self.run("tar xf %s" % self.tar_file, cwd=self.godir)
        return True
