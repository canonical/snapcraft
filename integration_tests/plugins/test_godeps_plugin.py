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
import subprocess

import fixtures
import testscenarios
from testtools.matchers import Equals, FileExists, Not

import integration_tests


class GodepsPluginTestCase(testscenarios.WithScenarios,
                           integration_tests.TestCase):

    scenarios = [
        ('no GOBIN', dict(set_gobin=False)),
        ('with GOBIN', dict(set_gobin=True)),
    ]

    def _assert_bcrypt_output(self, *, binary):
        hash_command = [binary, 'hash', '10', 'password']
        output = subprocess.check_output(hash_command)

        check_hash_command = [binary, 'check', output, 'password']
        output = subprocess.check_output(check_hash_command)

        self.assertThat(output.decode('UTF-8').strip(' \n'), Equals('Equal'))

    def test_stage(self):
        if self.set_gobin:
            gobin = 'gobin'
            self.useFixture(fixtures.EnvironmentVariable('GOBIN', gobin))

        self.run_snapcraft('stage', 'godeps')

        binary = os.path.join(self.stage_dir, 'bin', 'bcrypt')
        self.assertThat(binary, FileExists())

        self._assert_bcrypt_output(binary=binary)

    def test_stage_with_go_packages(self):
        if self.set_gobin:
            gobin = 'gobin'
            self.useFixture(fixtures.EnvironmentVariable('GOBIN', gobin))

        self.run_snapcraft('stage', 'godeps-with-go-packages')

        binary = os.path.join(self.stage_dir, 'bin', 'only-main')
        self.assertThat(binary, FileExists())
        self.assertThat(
            os.path.join(self.stage_dir, 'bin', 'bcrypt'),
            Not(FileExists()))

        self._assert_bcrypt_output(binary=binary)
