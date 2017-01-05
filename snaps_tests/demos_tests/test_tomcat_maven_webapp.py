# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016, 2017 Canonical Ltd
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

import snaps_tests


class TomcatMavenWebappTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'tomcat-maven-webapp'

    def test_tomcat_maven_webapp(self):
        if snapcraft.ProjectOptions().deb_arch == 'armhf':
            # https://bugs.launchpad.net/snapcraft/+bug/1647405
            self.skipTest('The maven plugin does not support armhf')
        snap_path = self.build_snap(self.snap_content_dir)
        snap_name = 'tomcat-webapp-demo'
        self.install_snap(snap_path, snap_name, '1.0')
        self.assert_service_running(snap_name, 'tomcat')
