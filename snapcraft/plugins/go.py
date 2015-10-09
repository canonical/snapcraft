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

PLUGIN_OPTIONS = {
    'source': '',
}


class GoPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('golang-go')

        if self.options.source.startswith("lp:"):
            self.fullname = self.options.source.split(":~")[1]
        else:
            self.fullname = self.options.source.split("://")[1]

    def env(self, root):
        # usr/lib/go/bin on newer Ubuntus, usr/bin on trusty
        return [
            "GOPATH={}/go".format(root),
        ]

    def pull(self):
        # use -d to only download (build will happen later)
        # use -t to also get the test-deps
        return self.run(['go', 'get', '-t', '-d', self.fullname])

    def build(self):
        if not self.run(['go', 'build', self.fullname]):
            return False
        if not self.run(['go', 'install', self.fullname]):
            return False
        return self.run(['cp', '-a', os.path.join(self.builddir, 'bin'),
                        self.installdir])

    def run(self, cmd, **kwargs):
        cmd = ['env', 'GOPATH=' + self.builddir] + cmd
        return super().run(cmd, **kwargs)
