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


class CopyPlugin(snapcraft.BasePlugin):

    def build(self):
        res = True
        for d in self.options.mkdirs:
            self.makedirs(os.path.join(self.installdir, d))
        for src, dst in self.options.files.items():
            res |= self.run(
                ["cp", "--preserve=all",
                 src, os.path.join(self.installdir, dst)],
                cwd=os.getcwd())
        return res
