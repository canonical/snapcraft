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

import glob
import inspect
import logging
import os
import re
import shutil
import subprocess
import sys

import fixtures
import pexpect
import testtools
from testtools import content
from testtools.matchers import (
    Contains,
    MatchesRegex
)

from snaps_tests import testbed

logger = logging.getLogger(__name__)

config = {}

_KVM_REDIRECT_PORTS = ['8080', '9000', '3000']


class CommandError(Exception):

    def __init__(self, command, working_dir, message):
        super().__init__()
        self.command = command
        self.working_dir = working_dir
        self.message = message

    def __str__(self):
        return 'Error running command {!r} in {!r}. {}'.format(
            self.command, self.working_dir, self.message)


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


class SnapsTestCase(testtools.TestCase):

    snap_content_dir = None

    def __init__(self, *args, **kwargs):
        # match base snap src path on current
        relative_path = os.path.relpath(
            os.path.dirname(inspect.getfile(self.__class__)),
            os.path.dirname(__file__))
        self.src_dir = os.path.join(*re.findall('(.*?)_tests/?(.*)',
                                                relative_path)[0])
        super().__init__(*args, **kwargs)

    def setUp(self):
        filter_ = config.get('filter', None)
        if filter_:
            if not re.match(filter_, self.snap_content_dir):
                self.skipTest(
                    '{} does not match the filter {}'.format(
                        self.snap_content_dir, filter_))
        logger.info('Testing {}'.format(self.snap_content_dir))
        super().setUp()
        if os.getenv('SNAPCRAFT_FROM_INSTALLED', False):
            self.snapcraft_command = 'snapcraft'
        else:
            self.snapcraft_command = os.path.join(
                os.getcwd(), 'bin', 'snapcraft')

        self.useFixture(fixtures.EnvironmentVariable('TERM', 'dumb'))

        temp_dir = fixtures.TempDir()
        self.useFixture(temp_dir)
        self.path = temp_dir.path

        self.snappy_testbed = None
        if not config.get('skip-install', False):
            ip = config.get('ip', None)
            if not ip or ip in ('localhost', '127.0.0.1'):
                self.snappy_testbed = testbed.LocalTestbed()
            else:
                port = config.get('port', None) or '22'
                proxy = config.get('proxy', None)
                self.snappy_testbed = testbed.SshTestbed(
                    ip, port, 'ubuntu', proxy)
            self.snappy_testbed.wait()

    def build_snap(self, snap_content_dir):
        project_dir = os.path.join(self.src_dir, snap_content_dir)
        tmp_project_dir = os.path.join(self.path, snap_content_dir)
        shutil.copytree(project_dir, tmp_project_dir, symlinks=True)

        self._snap(tmp_project_dir)

        snap_glob_path = os.path.join(tmp_project_dir,  '*.snap')
        return glob.glob(snap_glob_path)[0]

    def _snap(self, project_dir):
        command = '{} {}'.format(self.snapcraft_command, 'snap')
        self._run_command(
            command, project_dir, expect='Snapped .*\.snap', timeout=1200)

    def _run_command(
            self, command, working_dir, expect=pexpect.EOF, timeout=30):
        print(command)
        process = pexpect.spawn(
            command, cwd=working_dir, timeout=timeout)
        process.logfile_read = sys.stdout.buffer
        try:
            process.expect(expect)
        except pexpect.ExceptionPexpect:
            self._add_output_detail(process.before)
            raise CommandError(
                command, working_dir,
                'Expected output {!r} not found.'.format(expect)) from None
        finally:
            process.close()
            if process.exitstatus:
                self._add_output_detail(process.before)
                raise CommandError(
                    command, working_dir,
                    'Exit status: {!r}.'.format(process.exitstatus))

    def _add_output_detail(self, output):
        self.addDetail('output', content.text_content(str(output)))

    def install_snap(self, snap_local_path, snap_name, version,
                     devmode=False):
        if not config.get('skip-install', False):
            self.snappy_testbed.copy_file(snap_local_path, '/home/ubuntu')
            snap_file_name = os.path.basename(snap_local_path)
            snap_path_in_testbed = os.path.join(
                '/home/ubuntu/', snap_file_name)
            # Remove the snap file from the testbed.
            self.addCleanup(
                self.snappy_testbed.run_command,
                ['rm', snap_path_in_testbed])
            cmd = ['sudo', 'snap', 'install', '--force-dangerous',
                   snap_path_in_testbed]
            if devmode:
                cmd.append('--devmode')
            try:
                self.snappy_testbed.run_command(cmd)
            except subprocess.CalledProcessError as e:
                self.addDetail(
                    'ssh output', content.text_content(str(e.output)))
                raise
            # Uninstall the snap from the testbed.
            snap_name = snap_file_name[:snap_file_name.index('_')]
            self.addCleanup(
                self.snappy_testbed.run_command,
                ['sudo', 'snap', 'remove', snap_name])

            list_output = self.snappy_testbed.run_command(
                ['snap', 'list'])
            expected = '.*{}.*'.format(snap_name)
            self.assertThat(
                list_output, MatchesRegex(expected, flags=re.DOTALL))

    def assert_command_in_snappy_testbed(
            self, command, expected_output, cwd=None):
        if not config.get('skip-install', False):
            output = self.run_command_in_snappy_testbed(command, cwd)
            self.assertEqual(expected_output, output)

    def assert_command_in_snappy_testbed_with_regex(
            self, command, expected_regex, flags=0, cwd=None):
        if not config.get('skip-install', False):
            output = self.run_command_in_snappy_testbed(command, cwd)
            self.assertThat(output, MatchesRegex(expected_regex, flags=flags))

    def run_command_in_snappy_testbed(self, command, cwd=None):
        if not config.get('skip-install', False):
            try:
                return self.snappy_testbed.run_command(command, cwd)
            except subprocess.CalledProcessError as e:
                self._add_output_detail(e.output)
                raise

    def assert_service_running(self, snap, service):
        if not config.get('skip-install', False):
            output = self.run_command_in_snappy_testbed(
                ['systemctl', '--no-pager', 'status',
                 'snap.{}.{}'.format(snap, service)])
            expected = 'Active: active (running)'
            self.assertThat(output, Contains(expected))
