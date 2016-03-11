# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

import examples_tests


class TomcatMavenWebappTestCase(examples_tests.ExampleTestCase):

    example_dir = 'tomcat-maven-webapp'

    def test_tomcat_maven_webapp(self):
        self.build_snap(self.example_dir)
        snap_name = 'tomcat-webapp-demo'
        self.install_snap(self.example_dir, snap_name, '1.0')
        self.assert_service_running(snap_name, 'tomcat')
        expected = '.*<html.*>.*<title>Apache Tomcat/.*</title>.*</html>.*'
        self.assert_http_get(8080, expected)
