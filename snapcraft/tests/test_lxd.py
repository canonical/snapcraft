# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
from subprocess import CalledProcessError
from unittest.mock import (
    call,
    patch,
)

import fixtures

from snapcraft.internal import lxd
from snapcraft import tests
from snapcraft._options import ProjectOptions  # noqa


class LXDTestCase(tests.TestCase):

    @patch('snapcraft.internal.lxd.check_call')
    @patch('petname.Generate')
    def test_cleanbuild(self, mock_pet, mock_call):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        mock_pet.return_value = 'my-pet'

        project_options = ProjectOptions()
        lxd.Cleanbuilder('snap.snap', 'project.tar', project_options).execute()
        expected_arch = project_options.deb_arch

        self.assertEqual(
            'Setting up container with project assets\n'
            'Waiting for a network connection...\n'
            'Network connection established\n'
            'Retrieved snap.snap\n',
            fake_logger.output)

        mock_call.assert_has_calls([
            call(['lxc', 'launch', '-e',
                  'ubuntu:xenial/{}'.format(expected_arch),
                  'snapcraft-my-pet']),
            call(['lxc', 'file', 'push', 'project.tar',
                  'snapcraft-my-pet//root/project.tar']),
            call(['lxc', 'exec', 'snapcraft-my-pet', '--',
                  'tar', 'xvf', '/root/project.tar']),
            call(['lxc', 'exec', 'snapcraft-my-pet', '--',
                  'python3', '-c',
                  'import urllib.request; '
                  'urllib.request.urlopen('
                  '"http://start.ubuntu.com/connectivity-check.html", '
                  'timeout=5)']),
            call(['lxc', 'exec', 'snapcraft-my-pet', '--',
                  'apt-get', 'update']),
            call(['lxc', 'exec', 'snapcraft-my-pet', '--',
                  'apt-get', 'install', 'snapcraft', '-y']),
            call(['lxc', 'exec', 'snapcraft-my-pet', '--',
                  'snapcraft', 'snap', '--output', 'snap.snap']),
            call(['lxc', 'file', 'pull',
                  'snapcraft-my-pet//root/snap.snap',
                  'snap.snap']),
            call(['lxc', 'stop', '-f', 'snapcraft-my-pet']),
        ])

    @patch('snapcraft.internal.lxd.check_call')
    @patch('snapcraft.internal.lxd.sleep')
    def test_wait_for_network_loops(self, mock_sleep, mock_call):
        mock_call.side_effect = CalledProcessError(-1, ['my-cmd'])

        cb = lxd.Cleanbuilder('snap.snap', 'project.tar', 'amd64')

        raised = self.assertRaises(
            CalledProcessError,
            cb._wait_for_network)

        self.assertEqual(
            str(raised),
            "Command '['my-cmd']' returned non-zero exit status -1")

    @patch('snapcraft.internal.lxd.check_call')
    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_failed_build_with_debug(self, mock_sleep, mock_run, mock_call):
        call_list = []

        def run_effect(*args, **kwargs):
            call_list.append(args[0])
            if args[0] == ['snapcraft', 'snap', '--output', 'snap.snap']:
                raise CalledProcessError(returncode=255, cmd=args[0])

        mock_run.side_effect = run_effect

        project_options = ProjectOptions(debug=True)
        lxd.Cleanbuilder('snap.snap', 'project.tar', project_options).execute()

        self.assertIn(['bash', '-i'], call_list)

    @patch('snapcraft.internal.lxd.check_call')
    @patch('snapcraft.internal.lxd.Cleanbuilder._container_run')
    @patch('snapcraft.internal.lxd.sleep')
    def test_failed_build_without_debug(self, mock_sleep, mock_run, mock_call):
        call_list = []

        def run_effect(*args, **kwargs):
            call_list.append(args[0])
            if args[0] == ['snapcraft', 'snap', '--output', 'snap.snap']:
                raise CalledProcessError(returncode=255, cmd=args[0])

        mock_run.side_effect = run_effect

        project_options = ProjectOptions(debug=False)
        self.assertRaises(
            CalledProcessError,
            lxd.Cleanbuilder(
                'snap.snap', 'project.tar',
                project_options).execute)

        self.assertNotIn(['bash', '-i'], call_list)
