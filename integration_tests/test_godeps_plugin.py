# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import subprocess

import fixtures
import testscenarios

import integration_tests


class GodepsPluginTestCase(testscenarios.WithScenarios,
                           integration_tests.TestCase):

    scenarios = [
        ('no GOBIN', dict(set_gobin=False)),
        ('with GOBIN', dict(set_gobin=True)),
    ]

    def test_stage_go_plugin(self):
        if self.set_gobin:
            gobin = 'gobin'
            self.useFixture(fixtures.EnvironmentVariable('GOBIN', gobin))

        project_dir = 'simple-godeps'
        self.run_snapcraft('stage', project_dir)

        binary = os.path.join('stage', 'bin', 'bcrypt')
        hash_command = [binary, 'hash', '10', 'password']

        output = subprocess.check_output(
            hash_command, cwd=project_dir)

        check_hash_command = [binary, 'check', output, 'password']
        output = subprocess.check_output(
            check_hash_command, cwd=project_dir)

        self.assertEqual('Equal', output.decode('UTF-8').strip(' \n'))
