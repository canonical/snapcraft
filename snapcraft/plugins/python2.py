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

    _PLUGIN_STAGE_PACKAGES = [
        'python-dev',
    ]

    def __init__(self, name, options):
        if options.requirements:
            self.requirements = options.requirements
            self._PLUGIN_STAGE_PACKAGES.extend(['python-pkg-resources', 'python-setuptools'])
        else:
            self.requirements = None
        super().__init__(name, options)

    # note that we don't need to set PYTHONHOME here,
    # python discovers this automatically from it installed
    # location.  And PATH is automatically set by snapcraft.

    def pull(self):
        # A nice idea here would be to be asking setup tools
        # to use the deb layout, but that doesn't work with
        # prefix sadly

        if self.requirements and not (self.run(
                ['ln', '-s', os.path.join(self.installdir, 'usr', 'lib', 'python2.7', 'dist-packages'), os.path.join(self.installdir, 'usr', 'lib', 'python2.7', 'site-packages')]) and self.run(
                ['python2', os.path.join(self.installdir, 'usr', 'bin', 'easy_install'), '--prefix', os.path.join(self.installdir, 'usr'), 'pip']) and self.run(
                ['python2', os.path.join(self.installdir, 'usr', 'bin', 'pip2'), 'install', '--root', os.path.join(self.installdir, 'usr', 'lib', 'python2.7'), '--requirement', os.path.join(os.getcwd(), self.requirements)])):
            return False
        return True
