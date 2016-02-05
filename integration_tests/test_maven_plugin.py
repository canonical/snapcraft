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

import fixtures

import integration_tests


class MavenPluginTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable('SNAPCRAFT_SETUP_PROXIES', '1'))

    def test_build_maven_plugin(self):
        project_dir = 'simple-maven'
        self.run_snapcraft('build', project_dir)
