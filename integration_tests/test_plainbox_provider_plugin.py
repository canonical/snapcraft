# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import testscenarios
from testtools.matchers import FileExists

import integration_tests


class PlainboxProviderPluginTestCase(testscenarios.WithScenarios,
                                     integration_tests.TestCase):

    scenarios = [
        ('basic', dict(project_directory='plainbox-provider')),
        ('with-stage-packages',
         dict(project_directory='plainbox-provider-with-stage-packages')),
    ]

    def test_stage(self):
        self.run_snapcraft('stage', self.project_directory)

        self.assertThat(
            os.path.join(
                self.stage_dir, 'providers', 'simple-plainbox-provider',
                'plainbox-provider-simple.provider'),
            FileExists())
