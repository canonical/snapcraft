# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import snapcraft

import integration_tests


class MavenPluginTestCase(integration_tests.TestCase):

    def test_build_maven_plugin(self):
        if snapcraft.ProjectOptions().deb_arch == 'armhf':
            # https://bugs.launchpad.net/snapcraft/+bug/1647405
            self.skipTest('The maven plugin does not support armhf')
        self.run_snapcraft('build', 'maven-with-options')
