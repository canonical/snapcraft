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

import os

from xdg import BaseDirectory

import snapcraft


class SnapcraftCache:
    """Generic cache base class.

    This class is responsible for cache location, notification and pruning.
    """
    def __init__(self):
        self.cache_root = os.path.join(
            BaseDirectory.xdg_cache_home, 'snapcraft')

    def cache(self):
        raise NotImplementedError

    def prune(self, *args, **kwargs):
        raise NotImplementedError


class SnapcraftProjectCache(SnapcraftCache):
    """Project specific cache"""
    def __init__(self):
        super().__init__()
        self.config = snapcraft.internal.load_config()
        self.project_cache_root = os.path.join(
            self.cache_root, self.config.data['name'])
