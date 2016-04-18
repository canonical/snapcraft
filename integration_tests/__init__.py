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

import os
import shutil
import subprocess

import fixtures
import pexpect
import testtools
from testtools import content

from snapcraft.tests import (
    fixture_setup,
    test_config,
)


class TestCase(testtools.TestCase):

    def setUp(self):
        super().setUp()
        if os.getenv('SNAPCRAFT_FROM_INSTALLED', False):
            self.snapcraft_command = 'snapcraft'
        else:
            self.snapcraft_command = os.path.join(
                os.getcwd(), 'bin', 'snapcraft')

        self.snaps_dir = os.path.join(os.path.dirname(__file__), 'snaps')
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

        test_config.isolate_for_config(self)

    def run_snapcraft(self, command, project_dir=None):
        if isinstance(command, str):
            command = [command]
        if project_dir:
            if not os.path.exists(project_dir):
                cwd = self.copy_project_to_tmp(project_dir)
            else:
                cwd = os.path.join(self.path, project_dir)
        else:
            cwd = None
        try:
            return subprocess.check_output(
                [self.snapcraft_command] + command, cwd=cwd,
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise

    def copy_project_to_tmp(self, project_dir):
        tmp_project_dir = os.path.join(self.path, project_dir)
        shutil.copytree(
            os.path.join(self.snaps_dir, project_dir), tmp_project_dir)
        return tmp_project_dir

    def get_output_ignoring_non_zero_exit(self, binary, cwd):
        # Executing the binaries exists > 0 on trusty.
        # TODO investigate more to understand the cause.
        try:
            output = subprocess.check_output(
                binary, universal_newlines=True, cwd=cwd)
        except subprocess.CalledProcessError as exception:
            output = exception.output
        return output

    def login(self, email=None,
              password=None, expect_success=True):
        email = email or os.getenv('TEST_USER_EMAIL',
                                   'u1test+snapcraft@canonical.com')
        password = password or os.getenv('TEST_USER_PASSWORD', None)
        if not password:
            self.skipTest('No password provided for the test user.')

        process = pexpect.spawn(self.snapcraft_command, ['login'])
        process.expect_exact(
            'Enter your Ubuntu One SSO credentials.\r\n'
            'Email: ')
        process.sendline(email)
        process.expect_exact('Password: ')
        process.sendline(password)
        process.expect_exact(
            "One-time password (just press enter if you don't use two-factor "
            "authentication): ")
        process.sendline('')
        process.expect_exact('Authenticating against Ubuntu One SSO.')
        result = 'successful' if expect_success else 'failed'
        process.expect_exact('Login {}.'.format(result))

    def logout(self):
        output = self.run_snapcraft('logout')
        expected = ('Clearing credentials for Ubuntu One SSO.\n'
                    'Credentials cleared.\n')
        self.assertEqual(expected, output)

    def register_name(self, name):
        process = pexpect.spawn(self.snapcraft_command,
                                ['register-name', name])
        process.expect_exact('Registering {}.'.format(name))
        process.expect_exact(
            'Congrats! You\'re now the publisher for "{}".'.format(name))
