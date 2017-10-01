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
import glob
import os
import re
import shutil
import subprocess
import sys
import time
import uuid
import xdg
from distutils import dir_util

import fixtures
import pexpect
import requests
import testtools
import yaml
from unittest import mock
from testtools import content
from testtools.matchers import MatchesRegex

from snapcraft import ProjectOptions as _ProjectOptions
from snapcraft.internal.common import get_os_release_info
from snapcraft.tests import (
    fixture_setup,
    subprocess_utils
)


class RegisterError(Exception):
    pass


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

        self.deb_arch = _ProjectOptions().deb_arch
        self.distro_series = get_os_release_info()['VERSION_CODENAME']

    def run_snapcraft(
            self, command=None, project_dir=None, debug=True,
            pre_func=lambda: None, env=None):
        if project_dir:
            self.copy_project_to_cwd(project_dir)

        if command is None:
            command = []
        if isinstance(command, str):
            command = [command]
        snapcraft_command = [self.snapcraft_command]
        if debug:
            snapcraft_command.append('-d')
        try:
            pre_func()
            snapcraft_output = subprocess.check_output(
                snapcraft_command + command,
                stderr=subprocess.STDOUT, universal_newlines=True,
                env=env)
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

    def set_stage_package_version(
            self, snapcraft_yaml_path, part, package, version=None):
        return self.set_package_version(
            'stage-packages', snapcraft_yaml_path, part, package, version)

    def set_build_package_version(
            self, snapcraft_yaml_path, part, package, version=None):
        return self.set_package_version(
            'build-packages', snapcraft_yaml_path, part, package, version)

    def set_package_version(
            self, type_, snapcraft_yaml_path, part, package, version=None):
        # This doesn't handle complex package syntax.
        with open(snapcraft_yaml_path) as snapcraft_yaml_file:
            snapcraft_yaml = yaml.load(snapcraft_yaml_file)
        if part:
            packages = snapcraft_yaml['parts'][part].get(type_, [])
        else:
            packages = snapcraft_yaml.get(type_, [])
        for index, package_in_yaml in enumerate(packages):
            if package_in_yaml.split('=')[0] == package:
                if version is None:
                    version = get_package_version(
                        package, self.distro_series, self.deb_arch)

                packages[index] = '{}={}'.format(package, version)
                break
        else:
            self.fail("The part {} doesn't have a package {}".format(
                part, package))

        with open(snapcraft_yaml_path, 'w') as snapcraft_yaml_file:
            yaml.dump(snapcraft_yaml, snapcraft_yaml_file)
        return version

    def set_build_package_architecture(
            self, snapcraft_yaml_path, part, package, architecture):
        # This doesn't handle complex package syntax.
        with open(snapcraft_yaml_path) as snapcraft_yaml_file:
            snapcraft_yaml = yaml.load(snapcraft_yaml_file)
        packages = snapcraft_yaml['parts'][part]['build-packages']
        for index, package_in_yaml in enumerate(packages):
            if package_in_yaml == package:
                packages[index] = '{}:{}'.format(package, architecture)
                break
        else:
            self.fail("The part {} doesn't have a package {}".format(
                part, package))

        with open(snapcraft_yaml_path, 'w') as snapcraft_yaml_file:
            yaml.dump(snapcraft_yaml, snapcraft_yaml_file)


class BzrSourceBaseTestCase(TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('bzr') is None:
            self.skipTest('bzr is not installed')

    def init_source_control(self):
        subprocess.check_call(
            ['bzr', 'init', '.'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.check_call(
            ['bzr', 'whoami', '--branch', '"Example Dev <dev@example.com>"'])

    def commit(self, message, unchanged=False):
        command = ['bzr', 'commit', '-m', message]
        if unchanged:
            command.append('--unchanged')
        subprocess.check_call(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def get_revno(self, path=None):
        command = ['bzr', 'revno', '-r', '-1']
        if path:
            command.append(path)
        return subprocess.check_output(
            command, universal_newlines=True).strip()


class GitSourceBaseTestCase(TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('git') is None:
            self.skipTest('git is not installed')

    def init_source_control(self):
        subprocess.check_call(
            ['git', 'init', '.'], stdout=subprocess.DEVNULL)
        subprocess.check_call(
            ['git', 'config', '--local', 'user.name', '"Example Dev"'])
        subprocess.check_call(
            ['git', 'config', '--local', 'user.email', 'dev@example.com'])

    def add_file(self, file_path):
        subprocess.check_call(
            ['git', 'add', file_path], stdout=subprocess.DEVNULL)

    def commit(self, message, allow_empty=False):
        command = ['git', 'commit', '-m', message]
        if allow_empty:
            command.append('--allow-empty')
        subprocess.check_call(command, stdout=subprocess.DEVNULL)

    def tag(self, tag_name):
        subprocess.check_call(
            ['git', 'tag', '-a', '-m', tag_name, tag_name],
            stdout=subprocess.DEVNULL)

    def get_revno(self):
        return subprocess_utils.call_with_output(
            ['git', 'rev-list', 'HEAD', '--max-count=1'])


class HgSourceBaseTestCase(TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('hg') is None:
            self.skipTest('mercurial is not installed')

    def init_source_control(self):
        subprocess.check_call(['hg', 'init', '.'])

    def commit(self, message, file_):
        subprocess.check_call(
            ['hg', 'commit', '-m', message, '--user',
             '"Example Dev"', '-A', file_])

    def get_revno(self, path=None):
        command = ['hg', 'log', '--template', '"{desc}"', '-r', '-1']
        if path:
            command.extend(['--cwd', path])
        return subprocess.check_output(
            command, universal_newlines=True).strip()

    def get_id(self):
        return subprocess_utils.call_with_output(['hg', 'id']).split()[0]


class SubversionSourceBaseTestCase(TestCase):

    def setUp(self):
        super().setUp()
        if shutil.which('svn') is None:
            self.skipTest('svn is not installed')

    def init_source_control(self):
        subprocess.check_call(
            ['svnadmin', 'create', 'repo'], stdout=subprocess.DEVNULL)

    def checkout(self, source, destination):
        subprocess.check_call(
            ['svn', 'checkout', source, destination],
            stdout=subprocess.DEVNULL)

    def add(self, file_path, cwd=None):
        subprocess.check_call(
            ['svn', 'add', file_path], stdout=subprocess.DEVNULL, cwd=cwd)

    def commit(self, message, cwd=None):
        subprocess.check_call(
            ['svn', 'commit', '-m', message],
            stdout=subprocess.DEVNULL, cwd=cwd)

    def update(self, cwd=None):
        subprocess.check_call(
            ['svn', 'update'], stdout=subprocess.DEVNULL, cwd=cwd)


class StoreTestCase(TestCase):

    def setUp(self):
        super().setUp()
        self.test_store = fixture_setup.TestStore()
        self.useFixture(self.test_store)

    def is_store_fake(self):
        return (os.getenv('TEST_STORE') or 'fake') == 'fake'

    def is_store_staging(self):
        return os.getenv('TEST_STORE') == 'staging'

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
        process = pexpect.spawn(self.snapcraft_command, command)
        process.expect(r'.*\[y/N\]: ')
        process.sendline('y')
        try:
            process.expect_exact(
                'Congrats! You are now the publisher of {!r}.'.format(
                    snap_name))
        except pexpect.exceptions.EOF:
            wait_error_regex = (
                '.*You must wait (\d+) seconds before trying to register your '
                'next snap.*')
            output = process.before.decode(sys.getfilesystemencoding())
            match = re.search(wait_error_regex, output)
            if wait and match:
                time.sleep(int(match.group(1)))
                # This could get stuck for ever if the user is registering
                # other snaps in parallel.
                self.register(snap_name, private, wait)
            else:
                raise RegisterError(output)

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

    def get_unique_name(self, prefix='snapcrafttest'):
        """Return a unique snap name.

        It uses a UUIDv4 to create unique names and limits its full size
        to 40 chars (as defined in the snap specification).
        """
        unique_id = uuid.uuid4().int
        return '{}-{}'.format(prefix, unique_id)[:40]

    def get_unique_version(self):
        """Return a unique snap version.

        It uses a UUIDv4 to create unique version and limits its full size
        to 32 chars (as defined in the snap specification).
        """
        unique_id = uuid.uuid4().int
        return '{}'.format(unique_id)[:32]

    def update_name_arch_and_version(self, name=None, arch=None,
                                     version=None):
        if name is None:
            name = self.get_unique_name()
        if version is None:
            version = self.get_unique_version()
        if arch is None:
            arch = 'amd64'
        for line in fileinput.input(
                os.path.join('snap', 'snapcraft.yaml'), inplace=True):
            if 'name: ' in line:
                print('name: {}'.format(name))
            elif 'version: ' in line:
                print('version: {}'.format(version))
            elif 'architectures: ' in line:
                print('architectures: [{}]'.format(arch))
            else:
                print(line)

    def update_name_and_version(self, name=None, version=None):
        if name is None:
            name = self.get_unique_name()
        if version is None:
            version = self.get_unique_version()
        for line in fileinput.input(
                os.path.join('snap', 'snapcraft.yaml'), inplace=True):
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
                process.expect(
                    'Signing validations assertion for {}'.format(v))
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
                    'Build assertion .*{}-build saved to disk.'.format(
                        snap_filename))
            else:
                process.expect(
                    'Build assertion .*{}-build pushed.'.format(snap_filename))

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


class SnapdIntegrationTestCase(TestCase):

    def setUp(self):
        if os.environ.get('ADT_TEST') and self.deb_arch == 'armhf':
            self.skipTest("The autopkgtest armhf runners can't install snaps")
        super().setUp()

    def install_snap(self):
        try:
            subprocess.check_output(
                ['sudo', 'snap', 'install',
                 glob.glob('*.snap')[0],
                 '--dangerous'],
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise


def get_package_version(package_name, series, deb_arch):
    # http://people.canonical.com/~ubuntu-archive/madison.cgi?package=hello&a=amd64&c=&s=zesty&text=on
    params = {
        'package': package_name,
        's': '{0},{0}-updates,{0}-security'.format(series),
        'a': deb_arch,
        'text': 'on',
    }
    query = requests.get('http://people.canonical.com/~ubuntu-archive/'
                         'madison.cgi', params)
    query.raise_for_status()
    package = query.text.strip().split('\n')[-1]
    package_status = [i.strip() for i in package.strip().split('|')]
    return package_status[1]
