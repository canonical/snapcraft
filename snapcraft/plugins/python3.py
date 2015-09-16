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


class Python3Plugin(snapcraft.BasePlugin):

    _PLUGIN_STAGE_PACKAGES = [
        'python3-dev',
        'python3-setuptools',
    ]

    def __init__(self, name, options):
        if options.requirements:
            self.requirements = options.requirements
            self._PLUGIN_STAGE_PACKAGES.extend(['python3-pkg-resources', 'python3-setuptools'])
        else:
            self.requirements = None
        super().__init__(name, options)

    # note that we don't need to set PYTHONHOME here,
    # python discovers this automatically from it installed
    # location, see https://code.launchpad.net/~mvo/snapcraft/python3-project/+merge/264521/comments/664308
    #
    # PATH is automatically set by snapcraft

    def env(self, root):
        return ["PYTHONPATH=%s" % os.path.join(
            root, 'usr', 'lib', 'python3', 'dist-packages')]

    def pull(self):
        # A nice idea here would be to be asking setup tools
        # to use the deb layout, but that doesn't work with
        # prefix sadly

        if self.requirements and not (self.run(
                ['ln', '-s', os.path.join(self.installdir, 'usr', 'lib', 'python3', 'dist-packages'), os.path.join(self.installdir, 'usr', 'lib', 'python3.4', 'site-packages')]) and self.run(
                ['python3', os.path.join(self.installdir, 'usr', 'bin', 'easy_install3'), '--prefix', os.path.join(self.installdir, 'usr'), 'pip']) and self.run(
                ['python3', os.path.join(self.installdir, 'usr', 'bin', 'pip3'), 'install', '--target', os.path.join(self.installdir, 'usr', 'lib', 'python3.4', 'site-packages'), '--requirement', os.path.join(os.getcwd(), self.requirements)])):
            return False
        return True
