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

from snapcraft.plugins.make import MakePlugin

PLUGIN_OPTIONS = {
    'source': '',
    'source-type': '',
    'source-tag': '',
    'source-branch': '',
    'configflags': [],
}


class CMakePlugin(MakePlugin):

    _PLUGIN_BUILD_PACKAGES = [
        'cmake',
    ]

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('cmake')

    def build(self):
        return self.run(['cmake', '.', '-DCMAKE_INSTALL_PREFIX='] +
                        self.options.configflags) and super().build()
