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

    def env(self, root):
        return [
            "PATH=%s/usr/local/go/bin:$PATH" % root,
            "GOROOT=%s/usr/local/go" % root,
        ]

    def pull(self):
        return self.run(['wget', '-c', URLS[self.arch]])

    def build(self):
        tar_file = os.path.join(
            self.builddir, os.path.basename(URLS[self.arch]))
        targetdir = os.path.join(self.installdir, 'usr', 'local')
        self.makedirs(targetdir)
        return self.run(['tar', 'xf', tar_file], cwd=targetdir)

    def snapFiles(self):
        return ([], [])
