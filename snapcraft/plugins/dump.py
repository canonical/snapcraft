# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

"""This plugin just dumps the content from a specified source.

This plugin uses the common plugin keywords as well as those for 'sources'.
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

In the cases where dumping the content needs some mangling or organizing
one would take advantage of the core functionalities available to plugins
such as: `filesets`, `stage`, `snap` and `organize`.
"""

import os
import shutil

import snapcraft


class DumpPlugin(snapcraft.BasePlugin):

    def build(self):
        super().build()
        if os.path.exists(self.installdir):
            os.rmdir(self.installdir)
        if self.dest_subdir:
            shutil.copytree(self.builddir, self.installdir + os.sep + self.dest_subdir,
                        copy_function=snapcraft.common.link_or_copy)
        else:
            shutil.copytree(self.builddir, self.installdir,
                        copy_function=snapcraft.common.link_or_copy)
