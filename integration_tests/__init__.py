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
        self.useFixture(fixtures.EnvironmentVariable('TERM', 'dumb'))

    def run_snapcraft(self, command, project_dir=None, yaml_dir=None):
        if isinstance(command, str):
            command = [command]
        if project_dir:
            if not os.path.exists(project_dir):
                cwd = self.copy_project_to_tmp(project_dir)
            else:
                cwd = os.path.join(self.path, project_dir)
        else:
            cwd = None

        if yaml_dir:
            cwd = os.path.join(self.path, yaml_dir)

        try:
            snapcraft_output = subprocess.check_output(
                [self.snapcraft_command, '-d'] + command, cwd=cwd,
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise

        if not os.getenv('SNAPCRAFT_IGNORE_APT_AUTOREMOVE', False):
            self.addCleanup(self.run_apt_autoremove)

        return snapcraft_output

    def run_apt_autoremove(self):
        deb_env = os.environ.copy()
        deb_env.update({
            'DEBIAN_FRONTEND': 'noninteractive',
            'DEBCONF_NONINTERACTIVE_SEEN': 'true',
        })

        try:
            autoremove_output = subprocess.check_output(
                'sudo apt-get autoremove -y'.split(),
                stderr=subprocess.STDOUT, env=deb_env)
            self.addDetail(
                'apt-get autoremove output',
                content.text_content(autoremove_output.decode('utf-8')))
        except subprocess.CalledProcessError as e:
            self.addDetail(
                'apt-get autoremove error', content.text_content(str(e)))
            self.addDetail(
                'apt-get autoremove output',
                content.text_content(e.output.decode('utf-8')))

            if os.getenv('SNAPCRAFT_APT_AUTOREMOVE_CHECK_FAIL', False):
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
        if expect_success:
            process.expect_exact(
                'We strongly recommend enabling multi-factor authentication:')
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

    def register_key(self, key_name, email=None, password=None,
                     expect_success=True):
        email = email or self.test_store.user_email
        password = password or self.test_store.user_password

        process = pexpect.spawn(
            self.snapcraft_command, ['register-key', key_name])

        process.expect_exact(
            'Enter your Ubuntu One SSO credentials.\r\n'
            'Email: ')
        process.sendline(email)
        process.expect_exact('Password: ')
        process.sendline(password)
        if expect_success:
            process.expect_exact(
                'We strongly recommend enabling multi-factor authentication:')
            process.expect_exact('Login successful.')
            process.expect(
                r'Done\. The key "{}" .* may be used to sign your '
                r'assertions\.'.format(key_name))
        else:
            process.expect_exact('Login failed.')
            process.expect_exact(
                'Cannot continue without logging in successfully.')
        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus

    def list_keys(self, expected_keys):
        process = pexpect.spawn(self.snapcraft_command, ['list-keys'])

        for enabled, key_name, key_id in expected_keys:
            process.expect('{} *{} *{}'.format(
                '\*' if enabled else '-', key_name, key_id))
        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus

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

    def gated(self, snap_name, expected_validations=[], expected_output=None):
        process = pexpect.spawn(self.snapcraft_command, ['gated', snap_name])

        if expected_output:
            process.expect(expected_output)
        else:
            for name, revision in expected_validations:
                process.expect('{} *{}'.format(name, revision))
        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus

    def validate(self, snap_name, validations, expected_error=None):
        process = pexpect.spawn(self.snapcraft_command,
                                ['validate', snap_name] + validations)
        if expected_error:
            process.expect(expected_error)
        else:
            for v in validations:
                process.expect('Signing validation {}'.format(v))
        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus

    def sign_build(self, snap_filename, key_name='default', local=False,
                   expect_success=True):
        cmd = ['sign-build', snap_filename, '--key-name', key_name]
        if local:
            # only sign it, no pushing
            cmd.append('--local')
        process = pexpect.spawn(self.snapcraft_command, cmd)
        if expect_success:
            if local:
                process.expect(
                    'Build assertion {}-build saved to disk.'.format(
                        snap_filename))
            else:
                process.expect(
                    'Build assertion {}-build pushed.'.format(snap_filename))

        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus

    def close(self, *args, **kwargs):
        process = pexpect.spawn(
            self.snapcraft_command, ['close'] + list(args))
        expected = kwargs.get('expected')
        if expected is not None:
            process.expect(expected)
        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus

    def push(self, snap, release=None, expected=None):
        actions = ['push', snap]
        if release is not None:
            actions += ['--release', release]
        process = pexpect.spawn(
            self.snapcraft_command, actions)
        if expected is not None:
            process.expect(expected)
        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus
