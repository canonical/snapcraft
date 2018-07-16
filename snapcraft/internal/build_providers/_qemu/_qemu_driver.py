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

import logging
import shlex
import shutil
import socket
import subprocess
import sys
import telnetlib
import time
from typing import List, Sequence

import paramiko

from snapcraft.internal.build_providers import errors


logger = logging.getLogger(__name__)
# Avoid getting paramiko logs which are overly verbose.
logging.getLogger("paramiko").setLevel(logging.CRITICAL)


def _popen(command: List[str], **kwargs) -> subprocess.Popen:
    logger.debug("Running {}".format(" ".join(command)))
    return subprocess.Popen(command, **kwargs)


def _get_qemu_command() -> str:
    # TODO support more architectures.
    return "qemu-system-x86_64"


class QemuDriver:
    """A driver to interact with qemu virtual machines."""

    provider_name = "qemu"
    _PROJECT_DEV = "project_dev"
    _PROJECT_MOUNT = "project_mount"

    def __init__(self, *, ssh_username: str, ssh_key_file: str) -> None:
        """Initialize a QemuDriver instance.

        :raises errors.ProviderCommandNotFound:
            if the relevant qemu command is not found.
        """
        provider_cmd = _get_qemu_command()
        if not shutil.which(provider_cmd):
            raise errors.ProviderCommandNotFound(command=provider_cmd)
        self.provider_cmd = provider_cmd
        # TODO detect collisions and make dynamic
        self._telnet_port = 64444
        # TODO detect collisions and make dynamic
        self._ssh_port = 5555
        self._ssh_username = ssh_username
        self._ssh_key_file = ssh_key_file
        self._qemu_proc = None  # type: subprocess.Popen
        self._ssh_client = paramiko.SSHClient()
        self._ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    def launch(
        self,
        *,
        hda: str,
        qcow2_drives: Sequence,
        project_9p_dev: str,
        ram: str = None,
        enable_kvm: bool = True
    ) -> None:
        # TODO check for latest snapshot to launch for it instead of a cold
        #      boot.
        # TODO replace -nographic with a spinner and redirect to a log file.
        cmd = [
            "sudo",
            self.provider_cmd,
            "-m",
            ram,
            "-nographic",
            "-monitor",
            "telnet::{},server,nowait".format(self._telnet_port),
            "-hda",
            hda,
            "-fsdev",
            "local,id={},path={},security_model=none".format(
                self._PROJECT_DEV, project_9p_dev
            ),
            "-device",
            "virtio-9p-pci,fsdev={},mount_tag={}".format(
                self._PROJECT_DEV, self._PROJECT_MOUNT
            ),
            "-device",
            "e1000,netdev=net0",
            "-netdev",
            "user,id=net0,hostfwd=tcp::{}-:22".format(self._ssh_port),
        ]
        for drive in qcow2_drives:
            cmd.append("-drive")
            cmd.append("file={},if=virtio,format=qcow2".format(drive))
        if enable_kvm:
            cmd.append("-enable-kvm")

        # TODO we might want to spawn another thread here to keep an eye on
        #      the process. This is good enough for now.
        # TODO better encapsulation on setting the log file is needed.
        with open(".vm-builder.log", "w") as vm_builder_log:
            self._qemu_proc = _popen(cmd, stdout=vm_builder_log, stderr=vm_builder_log)
        # Check the immediate return code to make sure things haven't gone
        # wrong.
        proc_poll = self._qemu_proc.poll()
        if proc_poll is not None and proc_poll != 0:
            raise errors.ProviderLaunchError(
                provider_name=self.provider_name, exit_code=proc_poll
            )
        self._wait_for_ssh()

    def stop(self) -> None:
        self._ssh_client.close()
        try:
            telnet = telnetlib.Telnet(host="localhost", port=self._telnet_port)
        except socket.gaierror as telnet_error:
            raise errors.ProviderCommunicationError(
                protocol="telnet", port=self._telnet_port, error=telnet_error.strerror
            ) from telnet_error
        try:
            telnet.read_until("(qemu) ".encode())
            telnet.write("savevm latest\n".encode())
            telnet.read_until("(qemu) ".encode())
            telnet.write("quit\n".encode())
        except (OSError, EOFError) as telnet_error:
            raise errors.ProviderStopError(
                provider_name=self.provider_name, exit_code=None
            ) from telnet_error

    def execute(self, *, command: List[str]) -> None:
        # Properly quote and join the command
        command_string = " ".join([shlex.quote(c) for c in command])

        channel = self._ssh_client.get_transport().open_session()
        channel.get_pty()
        channel.exec_command(command_string)
        channel.settimeout(0.0)

        exit_code = None
        while exit_code is None:
            if channel.recv_ready():
                sys.stdout.write(channel.recv(1024).decode())
                sys.stdout.flush()
            if channel.exit_status_ready():
                exit_code = channel.recv_exit_status()
                logger.debug(
                    "Command {!r} returned exit code: {}".format(
                        command_string, exit_code
                    )
                )

        if exit_code != 0:
            raise errors.ProviderExecError(
                provider_name=self.provider_name, command=command, exit_code=exit_code
            )

    def _wait_for_ssh(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            while True:
                time.sleep(1)
                result = sock.connect_ex(("localhost", self._ssh_port))
                logger.debug(
                    "Pinging for ssh availability: port check {}".format(result)
                )
                if result == 0:
                    break

        while True:
            time.sleep(1)
            try:
                self._ssh_client.connect(
                    "localhost",
                    port=self._ssh_port,
                    username=self._ssh_username,
                    key_filename=self._ssh_key_file,
                )
                break
            except paramiko.SSHException as ssh_error:
                logger.debug("Pinging for ssh: {}".format(str(ssh_error)))
