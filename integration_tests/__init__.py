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

import fileinput
import os
import re
import subprocess
import time
import uuid
import xdg
from distutils import dir_util

import fixtures
import pexpect
from unittest import mock
import testtools
from testtools import content
from testtools.matchers import MatchesRegex

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
            'XDG_CACHE_HOME', os.path.join(self.path, '.cache')))
        self.useFixture(fixtures.EnvironmentVariable(
            'XDG_DATA_HOME', os.path.join(self.path, 'data')))
        self.useFixture(fixtures.EnvironmentVariable('TERM', 'dumb'))

        patcher = mock.patch(
            'xdg.BaseDirectory.xdg_config_home',
            new=os.path.join(self.path, '.config'))
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch(
            'xdg.BaseDirectory.xdg_data_home',
            new=os.path.join(self.path, 'data'))
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch(
            'xdg.BaseDirectory.xdg_cache_home',
            new=os.path.join(self.path, '.cache'))
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher_dirs = mock.patch(
            'xdg.BaseDirectory.xdg_config_dirs',
            new=[xdg.BaseDirectory.xdg_config_home])
        patcher_dirs.start()
        self.addCleanup(patcher_dirs.stop)

        patcher_dirs = mock.patch(
            'xdg.BaseDirectory.xdg_data_dirs',
            new=[xdg.BaseDirectory.xdg_data_home])
        patcher_dirs.start()
        self.addCleanup(patcher_dirs.stop)

        # Note that these directories won't exist when the test starts,
        # they might be created after calling the snapcraft command on the
        # project dir.
        self.parts_dir = 'parts'
        self.stage_dir = 'stage'
        self.prime_dir = 'prime'

    def run_snapcraft(
            self, command, project_dir=None, debug=True,
            pre_func=lambda: None):
        if project_dir:
            self.copy_project_to_cwd(project_dir)

        if isinstance(command, str):
            command = [command]
        snapcraft_command = [self.snapcraft_command]
        if debug:
            snapcraft_command.append('-d')
        try:
            pre_func()
            snapcraft_output = subprocess.check_output(
                snapcraft_command + command,
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise

        if not os.getenv('SNAPCRAFT_IGNORE_APT_AUTOREMOVE', False):
            self.addCleanup(self.run_apt_autoremove)

        return snapcraft_output

    def run_snapcraft_parser(self, arguments):
        try:
            snapcraft_output = subprocess.check_output(
                [self.snapcraft_parser_command, '-d'] + arguments,
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise
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

    def copy_project_to_cwd(self, project_dir):
        # Because cwd already exists, shutil.copytree would raise
        # FileExistsError. Use the lesser known distutils.dir_util.copy_tree
        dir_util.copy_tree(
            os.path.join(self.snaps_dir, project_dir), self.path,
            preserve_symlinks=True)

    def get_output_ignoring_non_zero_exit(self, binary, cwd=None):
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
        expected = (r'.*Clearing credentials for Ubuntu One SSO.\n'
                    r'Credentials cleared.\n.*')
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

    def register(self, snap_name, private=False, wait=True):
        command = ['register', snap_name]
        if private:
            command.append('--private')
        try:
            self.run_snapcraft(command)
        except subprocess.CalledProcessError as e:
            wait_error_regex = (
                '.*You must wait (\d+) seconds before trying to register your '
                'next snap.*')
            match = re.search(wait_error_regex, e.output)
            if wait and match:
                time.sleep(int(match.group(1)))
                # This could get stuck for ever if the user is registering
                # other snaps in parallel.
                self.register(snap_name, private, wait)
            else:
                raise

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

    def list_registered(self, expected_snaps):
        process = pexpect.spawn(self.snapcraft_command, ['list-registered'])

        for name, visibility, price, notes in expected_snaps:
            # Ignores 'since' to avoid confusion on fake and actual stores.
            process.expect(
                '{} *[T:\-\d]+Z *{} *{} *{}'.format(
                    name, visibility, price, notes))

        process.expect(pexpect.EOF)
        process.close()
        return process.exitstatus

    def update_name_arch_and_version(self, name=None, arch=None,
                                     version=None):
        unique_id = uuid.uuid4().int
        if name is None:
            name = 'u1test-{}'.format(unique_id)
        if version is None:
            # The maximum size is 32 chars.
            version = str(unique_id)[:32]
        if arch is None:
            arch = 'amd64'
        for line in fileinput.input('snapcraft.yaml', inplace=True):
            if 'name: ' in line:
                print('name: {}'.format(name))
            elif 'version: ' in line:
                print('version: {}'.format(version))
            elif 'architectures: ' in line:
                print('architectures: [{}]'.format(arch))
            else:
                print(line)

    def update_name_and_version(self, name=None, version=None):
        unique_id = uuid.uuid4().int
        if name is None:
            name = 'u1test-{}'.format(unique_id)
        if version is None:
            # The maximum size is 32 chars.
            version = str(unique_id)[:32]
        for line in fileinput.input('snapcraft.yaml', inplace=True):
            if 'name: ' in line:
                print('name: {}'.format(name))
            elif 'version: ' in line:
                print('version: {}'.format(version))
            else:
                print(line)

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
