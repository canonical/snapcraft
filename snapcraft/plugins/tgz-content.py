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


class TgzContentPlugin(snapcraft.BasePlugin):
    def __init__(self, name, options):
        super().__init__(name, options)
        self.partdir = os.path.join(os.getcwd(), "parts", self.name)
    def pull(self):
        self.run("wget -c %s " % self.options.source, cwd=self.partdir)
        tar_file = os.path.basename(self.partdir)
        return self.run("tar xf %s" % tar_file, cwd=self.partdir)
