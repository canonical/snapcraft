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

    def pull(self):
        return self.run(["wget", "-c", self.options.source], cwd=self.builddir)

    def build(self):
        tar_file = os.path.join(self.builddir, os.path.basename(self.options.source))
        return self.run(["tar", "xf", tar_file], cwd=self.installdir)
