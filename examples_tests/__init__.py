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

import atexit
import logging
import os
import re
import tempfile
import shutil
import subprocess

import fixtures
import testtools
from testtools import content
from testtools.matchers import MatchesRegex

from examples_tests import testbed

logger = logging.getLogger(__name__)

config = {}

_KVM_REDIRECT_PORTS = ['8080', '9000', '3000']


def _get_latest_ssh_private_key():
    """Return the latest private key in ~/.ssh.

    :returns:
        Path of the most-recently-modified private SSH key
    :raises LookupError:
        If no such key was found.

    This function tries to mimic the logic found in ``ubuntu-device-flash``.
    It will look for the most recently modified private key in the users' SSH
    configuration directory.
    """
    candidates = []
    ssh_dir = os.path.expanduser('~/.ssh/')
    for filename in os.listdir(ssh_dir):
        # Skip public keys, we want the private key
        if filename.endswith('.pub'):
            continue
        ssh_key = os.path.join(ssh_dir, filename)
        # Skip non-files
        if not os.path.isfile(ssh_key):
            continue
        # Ensure that it is a real ssh key
        with open(ssh_key, 'rb') as stream:
            if stream.readline() != b'-----BEGIN RSA PRIVATE KEY-----\n':
                continue
        candidates.append(ssh_key)
    # Sort the keys by modification time, pick the most recent key
    candidates.sort(key=lambda f: os.stat(f).st_mtime, reverse=True)
    logger.debug('Available ssh public keys: %r', candidates)
    if not candidates:
        raise LookupError('Unable to find any private ssh key')
    return candidates[0]


class ExampleTestCase(testtools.TestCase):

    example_dir = None

    def setUp(self):
        filter_ = config.get('filter', None)
        if filter_:
            if not re.match(filter_, self.example_dir):
                self.skipTest(
                    '{} does not match the filter {}'.format(
                        self.example_dir, filter_))

        super().setUp()
        # To measure coverage, a wrapper for the snapcraft binary might be set
        # in the environment variable.
        snapcraft_bin = os.getenv('SNAPCRAFT', 'snapcraft')
        self.snapcraft_command = os.path.join(
            os.getcwd(), 'bin', snapcraft_bin)

        self.useFixture(
            fixtures.EnvironmentVariable('SNAPCRAFT_SETUP_PROXIES', '1'))

        self.snappy_testbed = None
        if not config.get('skip-install', False):
            ip = config.get('ip', None)
            if not ip:
                self.snappy_testbed = self._set_up_qemu_testbed()
            else:
                port = config.get('port', None) or '22'
                self.snappy_testbed = testbed.SshTestbed(
                    ip, port, 'ubuntu')
            self.snappy_testbed.wait()

    def _set_up_qemu_testbed(self):
        private_key = _get_latest_ssh_private_key()
        snappy_image = config.get('snappy_image', None)
        if not snappy_image:
            temp_dir = tempfile.mkdtemp()
            snappy_image = testbed.create_snappy_image(temp_dir)
            # Store the image path in the config so it's only create once
            # per execution.
            config['snappy_image'] = snappy_image
            # Delete the image when the execution exits.
            atexit.register(shutil.rmtree, temp_dir)
        snappy_testbed = testbed.QemuTestbed(
            snappy_image, '8022', 'ubuntu', private_key, _KVM_REDIRECT_PORTS)
        snappy_testbed.create()
        self.addCleanup(snappy_testbed.delete)
        return snappy_testbed

    def build_snap(self, example_dir):
        working_dir = os.path.join('examples', example_dir)
        subprocess.check_call(
            [self.snapcraft_command, 'clean'], cwd=working_dir)
        try:
            subprocess.check_output(
                [self.snapcraft_command, 'snap'], cwd=working_dir,
                stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(str(e.output)))
            raise

    def install_snap(self, example_dir, snap_name, version):
        if not config.get('skip-install', False):
            snap_file_name = '{}_{}_amd64.snap'.format(
                snap_name, version)
            snap_local_path = os.path.join(
                'examples', example_dir, snap_file_name)
            self.snappy_testbed.copy_file(snap_local_path, '/home/ubuntu')
            # Remove the snap file from the testbed.
            self.addCleanup(
                self.snappy_testbed.run_command,
                ['rm', os.path.join('/home/ubuntu/', snap_file_name)])
            output = self.snappy_testbed.run_command([
                'sudo', 'snappy', 'install', '--allow-unauthenticated',
                snap_file_name])
            # Uninstall the snap from the testbed.
            snap_name = snap_file_name[:snap_file_name.index('_')]
            self.addCleanup(
                self.snappy_testbed.run_command,
                ['sudo', 'snappy', 'remove', snap_name])
            expected = (
                'Installing {}\n'.format(snap_file_name) +
                '.*' +
                '{} +.* +.* +sideload *\n'.format(snap_name) +
                '.*')
            self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

    def assert_command_in_snappy_testbed(self, command, expected_output):
        if not config.get('skip-install', False):
            output = self.run_command_in_snappy_testbed(command)
            self.assertEqual(output, expected_output)

    def run_command_in_snappy_testbed(self, command):
        if not config.get('skip-install', False):
            try:
                return self.snappy_testbed.run_command(command)
            except subprocess.CalledProcessError as e:
                self.addDetail(
                    'ssh output', content.text_content(str(e.output)))
                raise

    def assert_service_running(self, snap, service):
        if not config.get('skip-install', False):
            output = self.run_command_in_snappy_testbed(
                ['sudo', 'snappy', 'service', 'status', snap])
            expected = (
                'Snap\t+Service\t+State\n'
                '{}\t+{}\t+enabled; loaded; active \(running\)\n'.format(
                    snap, service))
            self.assertThat(output, MatchesRegex(expected))
