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


class TestSnapcraftExamples(testscenarios.TestWithScenarios):

    testbed_ip = 'localhost'

    scenarios = [
        ('py3-project', {
            'dir': 'libpipeline',
            'snap': 'pipelinetest_1.0_amd64.snap',
            'internal_tests_commands': [
                ('/apps/bin/pipelinetest.pipelinetest',
                 'running ls | grep c\n'
                 'custom libpipeline called\n'
                 'include\n')],
            'external_tests_commands': [],
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
        logger.info('Running the snappy image in kvm.')
        system = supbrocess.check_output(['uname', 'm']).strip().decode('utf8')
        cls.vm_process = subprocess.Popen(
            ['qemu-system-' + system, '-m', '512', '-nographic',
             '-monitor', 'none', '-serial', 'stdio',
             cls.image_path])

    @classmethod
    def tearDownClass(cls):
        cls.vm_process.kill()
        shutil.rmtree(cls.temp_dir)

    def setUp(self):
        super().setUp()
        self._wait_for_ssh()

    def _wait_for_ssh(self):
        timeout = 300
        sleep = 10
        while (timeout > 0):
            try:
                self.run_command_through_ssh(['echo', 'testing ssh'])
                break
            except subprocess.CalledProcessError:
                if timeout <= 0:
                    raise
                else:
                    time.sleep(sleep)
                timeout -= sleep

    def run_command_through_ssh(self, command):
        ssh_command = ['ssh', 'ubuntu@' + self.testbed_ip]
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
        scp_command = ['scp']
        scp_command.extend(self._get_ssh_options())
        scp_command.extend([snap_path, 'ubuntu@localhost:/home/ubuntu'])
        subprocess.check_call(scp_command)

    def install_snap(self, snap_file):
        self.run_command_through_ssh(['sudo', 'snappy', 'install', snap_file])

    def test_example(self):
        example_dir = os.path.join('examples', self.dir)
        self.build_snap(example_dir)
        self.copy_snap_to_testbed(os.path.join(example_dir, self.snap))
        self.install_snap(self.snap)
        for command, expected_result in self.internal_tests_commands:
            with self.subTest(command):
                output = self.run_command_through_ssh(command.split(' '))
                self.assertEqual(output, expected_result)
        for command, expected_result in self.external_tests_commands:
            with self.subTest(command):
                output = subprocess.check_output(command.split(' '))
                self.assertEqual(output, expected_result)
