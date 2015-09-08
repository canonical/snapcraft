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


class Python2Plugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        if options.requirements:
            self.requirements = options.requirements
            _PLUGIN_STAGE_PACKAGES.append('python-pip')
        else:
            self.requirements = None
        super().__init__(name, options, stage_packages=_PLUGIN_STAGE_PACKAGES)

    _PLUGIN_STAGE_PACKAGES = [
        'python-dev',
    ]

    # note that we don't need to set PYTHONHOME here,
    # python discovers this automatically from it installed
    # location.  And PATH is automatically set by snapcraft.

    def pull(self):
        self.run(['env'])
        if self.requirements and not self.run(
                ['python2', '-m', 'pip', 'install', '-r', os.path.join(os.getcwd(), self.requirements)]):
             return False
        return True
