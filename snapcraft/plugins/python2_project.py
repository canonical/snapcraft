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


class Python2ProjectPlugin(snapcraft.BasePlugin):

    # note that we don't need to setup env(), python figures it out
    # see python2.py for more details

    def pull(self):
        return self.pull_branch(self.options.source)

    def build(self):
        return self.run(
            ["python2", "setup.py", "install", "--install-layout=deb",
             "--prefix=%s/usr" % self.installdir])
