# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

"""The ninja plugin is useful for leveraging tooling that uses ninja.

This plugin always runs 'ninja install'.
"""

import os
import snapcraft
import snapcraft.common


class NinjaPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('ninja-build')

    def build(self):
        super().build()
        env = os.environ.copy()
        env['DESTDIR'] = self.installdir
        self.ninja('install', env)

    def ninja(self, cmd, env={}):
        self.run(['ninja', 'install'], env=env)
