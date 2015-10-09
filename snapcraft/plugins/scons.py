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

PLUGIN_OPTIONS = {
    'source': '',
    'source-type': '',
    'source-tag': '',
    'source-branch': '',
    'scons-options': [],
}


class SconsPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('scons')

    def pull(self):
        return self.handle_source_options()

    def build(self):
        env = os.environ.copy()
        env['DESTDIR'] = self.installdir
        return (self.run(['scons', ] + self.options.scons_options) and
                self.run(['scons', 'install'] + self.scons_options, env=env))
