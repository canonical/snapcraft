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

import socket
import telnetlib
from unittest import mock

import paramiko
from testtools.matchers import Equals

from snapcraft.internal.build_providers import errors
from snapcraft.internal.build_providers._qemu._qemu_driver import QemuDriver
from tests import unit


class QemuDriverBaseTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        # This return value is only of importance to command checking.
        patcher = mock.patch("shutil.which", return_value=["qemu-system"])
        self.which_mock = patcher.start()
        self.addCleanup(patcher.stop)


class QemuDriverGeneralTest(QemuDriverBaseTest):
    def test_provider_name(self):
        self.assertThat(QemuDriver.provider_name, Equals("qemu"))

    def test_multipass_command_missing_raises(self):
        self.which_mock.return_value = []

        self.assertRaises(
            errors.ProviderCommandNotFound,
            QemuDriver,
            ssh_key_file="id_rsa",
            ssh_username="builder",
        )


class QemuDriverPassthroughBaseTest(QemuDriverBaseTest):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("subprocess.Popen")
        self.popen_mock = patcher.start()
        self.popen_mock().poll.return_value = None
        self.addCleanup(patcher.stop)

        patcher = mock.patch("socket.socket", spec=socket.socket)
        self.socket_mock = patcher.start()
        self.socket_mock().__enter__().connect_ex.return_value = 0
        self.addCleanup(patcher.stop)

        patcher = mock.patch("telnetlib.Telnet", spec=telnetlib.Telnet)
        self.telnet_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("time.sleep")
        self.sleep_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.ssh_channel_mock = mock.MagicMock(spec=paramiko.Channel)
        self.ssh_channel_mock.recv.return_value = b""
        self.ssh_channel_mock.exit_status_ready.return_value = True
        self.ssh_channel_mock.recv_exit_status.return_value = 0

        patcher = mock.patch("paramiko.SSHClient", spec=paramiko.SSHClient)
        self.sshclient_mock = patcher.start()
        self.sshclient_mock().get_transport().open_session.return_value = (
            self.ssh_channel_mock
        )
        self.addCleanup(patcher.stop)

        self.qemu_driver = QemuDriver(
            ssh_key_file="fake-id_rsa", ssh_username="fake-username"
        )
        self.instance_name = "stub-instance"


class QemuDriverLaunchTest(QemuDriverPassthroughBaseTest):
    def test_launch(self):
        self.qemu_driver.launch(
            hda="test-vm.qcow2",
            qcow2_drives=["cloudlocal.qcow2"],
            project_9p_dev="/snapcraft-project",
            ram="1",
        )

        self.popen_mock.assert_has_calls(
            [
                mock.call(),
                mock.call(
                    [
                        "sudo",
                        self.qemu_driver.provider_cmd,
                        "-m",
                        "1",
                        "-nographic",
                        "-monitor",
                        "telnet::{},server,nowait".format(
                            self.qemu_driver._telnet_port
                        ),
                        "-hda",
                        "test-vm.qcow2",
                        "-fsdev",
                        "local,id=project_dev,path=/snapcraft-project,security_model=none",
                        "-device",
                        "virtio-9p-pci,fsdev=project_dev,mount_tag=project_mount",
                        "-device",
                        "e1000,netdev=net0",
                        "-netdev",
                        "user,id=net0,hostfwd=tcp::{}-:22".format(
                            self.qemu_driver._ssh_port
                        ),
                        "-drive",
                        "file=cloudlocal.qcow2,if=virtio,format=qcow2",
                        "-enable-kvm",
                    ],
                    stdout=mock.ANY,
                    stderr=mock.ANY,
                ),
                mock.call().poll(),
            ]
        )

    def test_launch_no_kvm(self):
        self.qemu_driver.launch(
            hda="test-vm.qcow2",
            qcow2_drives=["cloudlocal.qcow2"],
            project_9p_dev="/snapcraft-project",
            ram="1",
            enable_kvm=False,
        )

        self.popen_mock.assert_has_calls(
            [
                mock.call(),
                mock.call(
                    [
                        "sudo",
                        self.qemu_driver.provider_cmd,
                        "-m",
                        "1",
                        "-nographic",
                        "-monitor",
                        "telnet::{},server,nowait".format(
                            self.qemu_driver._telnet_port
                        ),
                        "-hda",
                        "test-vm.qcow2",
                        "-fsdev",
                        "local,id=project_dev,path=/snapcraft-project,security_model=none",
                        "-device",
                        "virtio-9p-pci,fsdev=project_dev,mount_tag=project_mount",
                        "-device",
                        "e1000,netdev=net0",
                        "-netdev",
                        "user,id=net0,hostfwd=tcp::{}-:22".format(
                            self.qemu_driver._ssh_port
                        ),
                        "-drive",
                        "file=cloudlocal.qcow2,if=virtio,format=qcow2",
                    ],
                    stdout=mock.ANY,
                    stderr=mock.ANY,
                ),
                mock.call().poll(),
            ]
        )

    def test_launch_waits_for_ssh(self):
        self.qemu_driver.launch(
            hda="test-vm.qcow2",
            qcow2_drives=["cloudlocal.qcow2"],
            project_9p_dev="/snapcraft-project",
            ram="1",
        )

        self.socket_mock().__enter__().connect_ex.assert_called_once_with(
            ("localhost", self.qemu_driver._ssh_port)
        )
        self.sshclient_mock().connect.assert_called_once_with(
            "localhost",
            port=self.qemu_driver._ssh_port,
            username="fake-username",
            key_filename="fake-id_rsa",
        )

    def test_launch_fails(self):
        self.popen_mock().poll.return_value = 2

        self.assertRaises(
            errors.ProviderLaunchError,
            self.qemu_driver.launch,
            hda="test-vm.qcow2",
            qcow2_drives=["cloudlocal.qcow2"],
            project_9p_dev="/snapcraft-project",
            ram="1",
        )


class QemuDriverStopTest(QemuDriverPassthroughBaseTest):
    def test_stop(self):
        self.qemu_driver.stop()

        self.telnet_mock.assert_called_once_with(
            host="localhost", port=self.qemu_driver._telnet_port
        )
        self.telnet_mock().read_until.assert_has_calls(
            [mock.call(b"(qemu) "), mock.call(b"(qemu) ")]
        )
        self.telnet_mock().write.assert_has_calls(
            [mock.call(b"savevm latest\n"), mock.call(b"quit\n")]
        )

    def test_stop_fails_due_bad_read(self):
        self.telnet_mock().read_until.side_effect = EOFError()

        self.assertRaises(errors.ProviderStopError, self.qemu_driver.stop)

    def test_stop_fails_due_bad_write(self):
        self.telnet_mock().write.side_effect = OSError()

        self.assertRaises(errors.ProviderStopError, self.qemu_driver.stop)


class QemuDriverExecuteTest(QemuDriverPassthroughBaseTest):
    def test_execute(self):
        self.qemu_driver.execute(command=["ls", "/project"])

    def test_execute_fails(self):
        self.ssh_channel_mock.recv_exit_status.return_value = 1

        self.assertRaises(
            errors.ProviderExecError, self.qemu_driver.execute, command=["ls", "/root"]
        )
