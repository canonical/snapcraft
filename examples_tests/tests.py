# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
import shutil
import subprocess
import tempfile
import time
import unittest

import testscenarios


logger = logging.getLogger(__name__)


def _get_ubuntu_version():
    return subprocess.check_output(
        ['lsb_release', '-cs']).decode('utf8').strip()


class TestSnapcraftExamples(testscenarios.TestWithScenarios):

    testbed_ip = 'localhost'
    testbed_port = '8022'

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
        ('qmldemo', {
            'dir': 'qmldemo',
            'name': 'qmldemo',
            'version': '1',
         }),
        ('ros', {
            'dir': 'ros',
            'name': 'ros-example',
            'version': '1.0',
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
        cls.image_path = os.path.join(cls.temp_dir, 'snappy.img')
        logger.info('Creating a snappy image to run the tests.')
        subprocess.check_call(
            ['sudo', 'ubuntu-device-flash', '--verbose',
             'core', 'rolling', '--channel', 'edge',
             '--output', cls.image_path, '--developer-mode'])
        logger.info('Running the snappy image in a virtual machine.')
        system = subprocess.check_output(['uname', '-m']).strip().decode('utf8')
        qemu_command = ('qemu-system-' + system +
            ' -m 512 -nographic -net user -net nic,model=virtio' +
            ' -drive file=' + cls.image_path +
            ',if=virtio -redir tcp:{}::22'.format(cls.testbed_port) +
            ' -monitor none -serial none')
        cls.vm_process = subprocess.Popen(qemu_command, shell=True)

    @classmethod
    def tearDownClass(cls):
        cls.vm_process.kill()
        shutil.rmtree(cls.temp_dir)

    def setUp(self):
        super().setUp()
        self._wait_for_ssh()

    def _wait_for_ssh(self):
        logger.info('Waiting for ssh to be enable in the testbed...')
        timeout = 300
        sleep = 10
        while (timeout > 0):
            try:
                self.run_command_through_ssh(['echo', 'testing ssh'])
                break
            except subprocess.CalledProcessError:
                if timeout <= 0:
                    logger.error('Timed out waiting for ssh in the testbed.')
                    raise
                else:
                    time.sleep(sleep)
                timeout -= sleep

    def run_command_through_ssh(self, command):
        ssh_command = ['ssh', 'ubuntu@' + self.testbed_ip, '-p', self.testbed_port]
        ssh_command.extend(self._get_ssh_options())
        ssh_command.extend(command)
        return subprocess.check_output(ssh_command).decode("utf-8")

    def _get_ssh_options(self):
        return [
            '-o', 'UserKnownHostsFile=/dev/null',
            '-o', 'StrictHostKeyChecking=no',
            '-i', os.path.join(os.getenv('HOME'), '.ssh', 'id_rsa')
        ]

    def build_snap(self, project_dir):
        snapcraft = os.path.join(os.getcwd(), 'bin/snapcraft')
        subprocess.check_call([snapcraft, 'clean'], cwd=project_dir)
        subprocess.check_call(snapcraft, cwd=project_dir)

    def copy_snap_to_testbed(self, snap_path):
        scp_command = ['scp', '-P', self.testbed_port]
        scp_command.extend(self._get_ssh_options())
        scp_command.extend([snap_path, 'ubuntu@localhost:/home/ubuntu'])
        subprocess.check_call(scp_command)

    def delete_snap_from_testbed(self, snap_file_name):
        self.run_command_through_ssh(
            ['rm', os.path.join('/home/ubuntu/', snap_file_name)])

    def install_snap(self, snap_file_name):
        self.run_command_through_ssh(
            ['sudo', 'snappy', 'install', snap_file_name])

    def remove_snap(self, snap_name):
        self.run_command_through_ssh(['sudo', 'snappy', 'remove', snap_name])

    def test_example(self):
        if self.name == 'qmldemo' and _get_ubuntu_version() == 'trusty':
            self.skipTest('qmldemo is not supported on trusty.')

        example_dir = os.path.join('examples', self.dir)
        self.build_snap(example_dir)
        snap = '{}_{}_amd64.snap'.format(self.name, self.version)
        self.copy_snap_to_testbed(os.path.join(example_dir, snap))
        self.addCleanup(self.delete_snap_from_testbed, snap)
        self.install_snap(snap)
        self.addCleanup(self.remove_snap, self.name)
        if getattr(self, 'internal_tests_commads', None):
            for command, expected_result in self.internal_tests_commands:
                with self.subTest(command):
                    output = self.run_command_through_ssh(command.split(' '))
                    self.assertEqual(output, expected_result)
        if getattr(self, 'external_tests_commands', None):
            for command, expected_result in self.external_tests_commands:
                with self.subTest(command):
                    output = subprocess.check_output(command.split(' '))
                    self.assertEqual(output, expected_result)
