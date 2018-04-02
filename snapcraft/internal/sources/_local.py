# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

import copy
import glob
import os

from snapcraft import file_utils
from snapcraft.internal import common
from ._base import Base


class Local(Base):

    def pull(self):
        current_dir = os.getcwd()
        source_abspath = os.path.abspath(self.source)

        def ignore(directory, files):
            if directory == source_abspath or \
               directory == current_dir:
                ignored = copy.copy(common.SNAPCRAFT_FILES)
                snaps = glob.glob(os.path.join(directory, '*.snap'))
                if snaps:
                    snaps = [os.path.basename(s) for s in snaps]
                    ignored += snaps
                return ignored
            else:
                return []

        file_utils.link_or_copy_tree(
            source_abspath, self.source_dir, ignore=ignore)
