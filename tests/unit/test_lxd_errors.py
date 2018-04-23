# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
from testtools.matchers import Equals

from snapcraft.internal.lxd import _errors as errors
from tests import unit


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
        ('ContainerNetworkError', {
            'exception': errors.ContainerNetworkError,
            'kwargs': {'url': 'test'},
            'expected_message': (
                "Failed to get a network connection in the container: "
                "Could not successfully ping 'test'. "
                "If using a proxy, check its configuration.\n"
                "Refer to the documentation at "
                "https://linuxcontainers.org/lxd/getting-started-cli.")}),
        ('ContainerArchitectureError', {
            'exception': errors.ContainerArchitectureError,
            'kwargs': {'lxc_info': 'test'},
            'expected_message': (
                "Failed to detect container architecture: "
                "The output from 'lxc info' could not be read:\ntest")}),
        ('ContainerLXDNotInstalledError', {
            'exception': errors.ContainerLXDNotInstalledError,
            'kwargs': {},
            'expected_message': (
                "Failed to initialize container: "
                "LXD could not be found. "
                "You must have LXD installed in order to use cleanbuild.\n"
                "Refer to the documentation at "
                "https://linuxcontainers.org/lxd/getting-started-cli.")}),
        ('ContainerLXDSetupError', {
            'exception': errors.ContainerLXDSetupError,
            'kwargs': {},
            'expected_message': (
                "Failed to initialize container: "
                "Something seems to be wrong with your installation of LXD.\n"
                "Refer to the documentation at "
                "https://linuxcontainers.org/lxd/getting-started-cli.")}),
        ('ContainerLXDRemoteNotFoundError', {
            'exception': errors.ContainerLXDRemoteNotFoundError,
            'kwargs': {'remote': 'test'},
            'expected_message': (
                "Failed to initialize container: "
                "There are either no permissions or the remote 'test' "
                "does not exist. "
                "Verify the existing remotes by running `lxc remote list`.\n"
                "Refer to the documentation at "
                "https://linuxcontainers.org/lxd/getting-started-cli.")}),
        ('ContainerStartFailedError', {
            'exception': errors.ContainerStartFailedError,
            'kwargs': {},
            'expected_message': (
                "Failed to start container: "
                "The local folder could not be mounted into the container. "
                "The files /etc/subuid and /etc/subgid need to "
                "contain this line for mounting the local folder:\n"
                "    root:1000:1\n"
                "Note: Add the line to both files, do not "
                "remove any existing lines.\n"
                "Restart LXD after making this change.\n")}),
        ('ContainerCreationFailedError', {
            'exception': errors.ContainerCreationFailedError,
            'kwargs': {},
            'expected_message': (
                "Failed to create container: "
                "A new LXD container could not be created.\n"
                "Refer to the documentation at "
                "https://linuxcontainers.org/lxd/getting-started-cli.")}),
        ('ContainerRunError string', {
            'exception': errors.ContainerRunError,
            'kwargs': {
                'command': 'test-command',
                'exit_code': '1'
            },
            'expected_message': (
                "The following command failed to run: "
                "'test-command' exited with 1\n")}),
        ('ContainerRunError list', {
            'exception': errors.ContainerRunError,
            'kwargs': {
                'command': ['test-command', 'test-argument'],
                'exit_code': '1'
            },
            'expected_message': (
                "The following command failed to run: "
                "'test-command test-argument' exited with 1\n")}),
        ('ContainerSnapcraftCmdError string', {
            'exception': errors.ContainerSnapcraftCmdError,
            'kwargs': {
                'command': 'test-command',
                'exit_code': '1'
            },
            'expected_message': (
                "Snapcraft command failed in the container: "
                "'test-command' exited with 1\n")}),
        ('ContainerSnapcraftCmdError list', {
            'exception': errors.ContainerSnapcraftCmdError,
            'kwargs': {
                'command': ['test-command', 'test-argument'],
                'exit_code': '1'
            },
            'expected_message': (
                "Snapcraft command failed in the container: "
                "'test-command test-argument' exited with 1\n")}),
        ('SnapdQueryError', {
            'exception': errors.SnapdQueryError,
            'kwargs': {'snap': 'test', 'message': 'foo'},
            'expected_message': (
                "Failed to get snap in container: "
                "Error querying 'test' snap: foo"
                )}),
        ('SnapdConnectionError', {
            'exception': errors.SnapdConnectionError,
            'kwargs': {'api': 'test'},
            'expected_message': (
                "Failed to get snap in container: "
                "Could not connect to 'test'.")}),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)),
            Equals(self.expected_message))
