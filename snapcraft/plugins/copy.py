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
        for src in sorted(self.options.files):
            dst = self.options.files[src]
            if not os.path.lexists(src):
                snapcraft.common.log("WARNING: file '%s' missing" % src)
                res = False
                continue
            dst = os.path.join(self.installdir, dst)
            dst_dir = os.path.dirname(dst)
            if not os.path.exists(dst_dir):
                os.makedirs(dst_dir)
            res &= self.run(["cp", "--preserve=all", src, dst], cwd=os.getcwd())
        return res
