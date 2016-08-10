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

import fileinput
import os
import shutil
import subprocess
import time
import uuid

import fixtures
import pexpect
import testtools
from testtools import content

from snapcraft.tests import fixture_setup


class TestCase(testtools.TestCase):

    def setUp(self):
        super().setUp()
        if os.getenv('SNAPCRAFT_FROM_INSTALLED', False):
            self.snapcraft_command = 'snapcraft'
            self.snapcraft_parser_command = 'snapcraft-parser'
        else:
            self.snapcraft_command = os.path.join(
                os.getcwd(), 'bin', 'snapcraft')
            self.snapcraft_parser_command = os.path.join(
                os.getcwd(), 'bin', 'snapcraft-parser')

        self.snaps_dir = os.path.join(os.path.dirname(__file__), 'snaps')
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path

        self.useFixture(fixtures.EnvironmentVariable(
            'XDG_CONFIG_HOME', os.path.join(self.path, '.config')))
        self.useFixture(fixtures.EnvironmentVariable(
            'XDG_DATA_HOME', os.path.join(self.path, 'data')))

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
                [self.snapcraft_command, '-d'] + command, cwd=cwd,
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise

    def copy_project_to_tmp(self, project_dir):
        tmp_project_dir = os.path.join(self.path, project_dir)
        shutil.copytree(
            os.path.join(self.snaps_dir, project_dir), tmp_project_dir,
            symlinks=True)
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


class StoreTestCase(TestCase):

    def setUp(self):
        super().setUp()
        self.test_store = fixture_setup.TestStore()
        self.useFixture(self.test_store)

    def login(self, email=None, password=None, expect_success=True):
        email = email or self.test_store.user_email
        password = password or self.test_store.user_password

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

    def register(self, snap_name, wait=True):
        self.run_snapcraft(['register', snap_name])
        # sleep a few seconds to avoid hitting the store restriction on
        # following registrations.
        if wait:
            time.sleep(self.test_store.register_delay)

    def update_name_and_version(self, project_dir, name=None, version=None):
        unique_id = uuid.uuid4().int
        if name is None:
            name = 'u1test-{}'.format(unique_id)
        if version is None:
            # The maximum size is 32 chars.
            version = str(unique_id)[:32]
        updated_project_dir = self.copy_project_to_tmp(project_dir)
        yaml_file = os.path.join(project_dir, 'snapcraft.yaml')
        for line in fileinput.input(yaml_file, inplace=True):
            if 'name: ' in line:
                print('name: {}'.format(name))
            elif 'version: ' in line:
                print('version: {}'.format(version))
            else:
                print(line)
        return updated_project_dir
