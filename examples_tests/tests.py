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

import logging
import os
import platform
import re
import shutil
import subprocess
import tempfile
import time

import testscenarios
from testtools import content


logger = logging.getLogger(__name__)
config = {}


def _start_snappy_testbed(directory, ssh_port):
    logger.info('Creating a snappy image to run the tests.')

    image_path = os.path.join(directory, 'snappy.img')
    subprocess.check_call(
        ['sudo', 'ubuntu-device-flash', '--verbose',
         'core', '15.04', '--channel', 'stable',
         '--output', image_path, '--developer-mode'])
    logger.info('Running the snappy image in a virtual machine.')
    qemu_command = (
        'qemu-system-{}' +
        ' -enable-kvm' +
        ' -m 512 -nographic -net user -net nic,model=virtio' +
        ' -drive file={}' +
        ',if=virtio -redir tcp:{}::22' +
        ' -monitor none -serial none').format(
        platform.machine(), image_path, ssh_port)
    return subprocess.Popen(qemu_command, shell=True)


def _wait_for_ssh(ip, port, user, timeout, sleep):
    logger.info('Waiting for ssh to be enable in the testbed...')
    while (timeout > 0):
        try:
            _run_command_through_ssh(
                ip, port, user, ['echo', 'testing ssh'])
            break
        except subprocess.CalledProcessError:
            if timeout <= 0:
                logger.error('Timed out waiting for ssh in the testbed.')
                raise
            else:
                time.sleep(sleep)
                timeout -= sleep


def _run_command_through_ssh(ip, port, user, command):
    ssh_command = ['ssh', '-l', user, '-p', port, ip]
    ssh_command.extend(_get_ssh_options())
    ssh_command.extend(command)
    return subprocess.check_output(
        ssh_command, stderr=subprocess.STDOUT, universal_newlines=True)


def _get_ssh_options():
    return [
        '-o', 'UserKnownHostsFile=/dev/null',
        '-o', 'StrictHostKeyChecking=no',
        '-i', _get_latest_ssh_private_key()
    ]


def _get_latest_ssh_private_key():
    """Return the latest private key in ~/.ssh.

    :returns:
        Path of the most-recently-modified private SSH key
    :raises LookupError:
        If no such key was found.

    This function tries to mimic the logic found in ``ubuntu-device-flash``. It
    will look for the most recently modified private key in the users' SSH
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


def _copy_file_through_ssh(ip, port, user, local_file_path, remote_file_path):
    scp_command = ['scp', '-P', port]
    scp_command.extend(_get_ssh_options())
    scp_command.extend([
        local_file_path, '{}@{}:{}'.format(user, ip, remote_file_path)])
    subprocess.check_call(scp_command)


class TestSnapcraftExamples(testscenarios.TestWithScenarios):

    vm_process = None

    scenarios = [
        ('downloader-with-wiki-parts', {
            'dir': 'downloader-with-wiki-parts',
            'name': 'downloader',
            'version': '1.0',
            }),
        ('godd', {
            'dir': 'godd',
            'name': 'godd',
            'version': '1.0',
            }),
        ('gopaste', {
            'dir': 'gopaste',
            'name': 'gopaste',
            'version': '1.0',
            }),
        ('java-hello-world', {
            'dir': 'java-hello-world',
            'name': 'java-hello-world',
            'version': '0',
            }),
        ('libpipeline', {
            'dir': 'libpipeline',
            'name': 'pipelinetest',
            'version': '1.0',
            'internal_tests_commands': [
                ('/apps/bin/pipelinetest.pipelinetest',
                 'running ls | grep c\n'
                 'custom libpipeline called\n'
                 'include\n')],
            }),
        ('opencv', {
            'dir': 'opencv',
            'name': 'opencv-example',
            'version': '1.0',
            }),
        ('py2-project', {
            'dir': 'py2-project',
            'name': 'spongeshaker',
            'version': '0',
            }),
        ('py3-project', {
            'dir': 'py3-project',
            'name': 'spongeshaker',
            'version': '0',
            }),
        ('ros', {
            'dir': 'ros',
            'name': 'ros-example',
            'version': '1.0',
            'external_tests_commands': [
                # check that the hardcoded /usr/bin/python in rosversion
                # is changed to using /usr/bin/env python
                ("sed -n '/env/p;1q' snap/usr/bin/rosversion",
                 b'#!/usr/bin/env python\n',)],
            }),
        ('shout', {
            'dir': 'shout',
            'name': 'shout',
            'version': '0.52.0',
            }),
        ('tomcat-maven-webapp', {
            'dir': 'tomcat-maven-webapp',
            'name': 'tomcat-webapp-demo',
            'version': '1.0',
            }),
        ('webcam-webui', {
            'dir': 'webcam-webui',
            'name': 'webcam-webui',
            'version': '1',
            }),
        ('webchat', {
            'dir': 'webchat',
            'name': 'webchat',
            'version': '0.0.1',
            }),
    ]

    @classmethod
    def setUpClass(cls):
        cls.temp_dir = tempfile.mkdtemp()
        if not config.get('skip-install', False):
            ip = config.get('ip', None)
            if ip is None:
                cls.testbed_ip = 'localhost'
                cls.testbed_port = '8022'
                cls.vm_process = _start_snappy_testbed(
                    cls.temp_dir, cls.testbed_port)
            else:
                cls.testbed_ip = ip
                cls.testbed_port = config.get('port', '22')

            _wait_for_ssh(
                cls.testbed_ip, cls.testbed_port, 'ubuntu',
                timeout=300, sleep=10)

    @classmethod
    def tearDownClass(cls):
        if cls.vm_process:
            cls.vm_process.kill()
        shutil.rmtree(cls.temp_dir)

    def setUp(self):
        super().setUp()
        # To measure coverage, a wrapper for the snapcraft binary might be set
        # in the environment variable.
        snapcraft_bin = os.getenv('SNAPCRAFT', 'snapcraft')
        self.snapcraft_command = os.path.join(
            os.getcwd(), 'bin', snapcraft_bin)

    def run_command_through_ssh(self, command):
        try:
            return _run_command_through_ssh(
                self.testbed_ip, self.testbed_port, 'ubuntu', command)
        except subprocess.CalledProcessError as e:
            self.addDetail('ssh output', content.text_content(e.output))
            raise

    def build_snap(self, project_dir):
        subprocess.check_call(
            [self.snapcraft_command, 'clean'], cwd=project_dir)
        try:
            subprocess.check_output(
                [self.snapcraft_command, 'snap'], cwd=project_dir,
                stderr=subprocess.STDOUT, universal_newlines=True)
        except subprocess.CalledProcessError as e:
            self.addDetail('output', content.text_content(e.output))
            raise

    def copy_snap_to_testbed(self, snap_local_path):
        _copy_file_through_ssh(
            self.testbed_ip, self.testbed_port, 'ubuntu',
            snap_local_path, '/home/ubuntu/')

    def delete_snap_from_testbed(self, snap_file_name):
        self.run_command_through_ssh(
            ['rm', os.path.join('/home/ubuntu', snap_file_name)])

    def install_snap(self, snap_file_name):
        self.run_command_through_ssh([
            'sudo', 'snappy', 'install', '--allow-unauthenticated',
            snap_file_name])

    def remove_snap(self, snap_name):
        self.run_command_through_ssh(['sudo', 'snappy', 'remove', snap_name])

    def test_example(self):
        filter_ = config.get('filter', False)
        if filter_:
            if not re.match(filter_, self.dir):
                self.skipTest(
                    '{} does not match the filter {}'.format(
                        self.dir, filter_))

        example_dir = os.path.join('examples', self.dir)
        # Build snap will raise an exception in case of error.
        self.build_snap(example_dir)

        if not config.get('skip-install', False):
            if getattr(self, 'required-snaps', None):
                for snap in self.required_snaps:
                    self.install_snap(snap)
                    self.addCleanup(self.remove_snap, snap)

            snap_file_name = '{}_{}_amd64.snap'.format(self.name, self.version)
            self.copy_snap_to_testbed(
                os.path.join(example_dir, snap_file_name))
            self.addCleanup(self.delete_snap_from_testbed, snap_file_name)
            # Install snap will raise an exception in case of error.
            self.install_snap(snap_file_name)
            self.addCleanup(self.remove_snap, self.name)

            if getattr(self, 'internal_tests_commads', None):
                self._run_internal_commands(self.internal_tests_commands)

            if getattr(self, 'external_tests_commands', None):
                self._run_external_commands(self.external_tests_commands,
                                            example_dir)

    def _run_internal_commands(self, internal_tests_commands):
        for command, expected_result in internal_tests_commands:
            with self.subTest(command):
                output = self.run_command_through_ssh(command.split(' '))
                self.assertEqual(output, expected_result)

    def _run_external_commands(self, external_tests_commands, cwd=None):
        for command, expected_result in external_tests_commands:
            with self.subTest(command):
                output = subprocess.check_output(command, cwd=cwd, shell=True)
                self.assertEqual(output, expected_result)
