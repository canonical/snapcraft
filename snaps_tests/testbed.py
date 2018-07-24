# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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
import platform
import shutil
import subprocess
import time


logger = logging.getLogger(__name__)


class LocalTestbed:
    def wait(self):
        pass

    def run_command(self, command, cwd=None):
        if isinstance(command, list):
            command = " ".join(command)
        if not cwd:
            logger.warning("Running from /tmp")
            cwd = "/tmp"
        return subprocess.check_output(
            command, shell=True, cwd=cwd, stderr=subprocess.STDOUT
        ).decode("utf-8")

    def run_command_in_background(self, command, cwd=None):
        if not cwd:
            logger.warning("Running from /tmp")
            cwd = "/tmp"
        return subprocess.Popen(command, cwd=cwd)

    def copy_file(self, src, dst):
        shutil.copy(src, dst)


class SshTestbed:
    def __init__(self, ip, port, user, proxy=None, private_key=None):
        super().__init__()
        self.ip = ip
        self.port = port
        self.user = user
        self.proxy = proxy
        self.private_key = private_key

    def wait(self, timeout=300, sleep=10):
        logger.debug("Waiting for ssh to be enabled in the testbed...")
        while timeout > 0:
            try:
                self.run_command(["echo", "testing ssh"])
                break
            except subprocess.CalledProcessError:
                if timeout <= 0:
                    logger.error("Timed out waiting for ssh in the testbed.")
                    raise
                else:
                    time.sleep(sleep)
                    timeout -= sleep

    def run_command(self, command, cwd=None):
        ssh_command = self._prepare_ssh_command(command)
        return subprocess.check_output(ssh_command, stderr=subprocess.STDOUT).decode(
            "utf-8"
        )

    def _prepare_ssh_command(self, command):
        if isinstance(command, str):
            command = [command]
        if self.proxy:
            command = [
                "http_proxy={}".format(self.proxy),
                "https_proxy={}".format(self.proxy),
            ] + command
        ssh_command = ["ssh", "-l", self.user, "-p", self.port, self.ip]
        ssh_command.extend(self._get_options())
        ssh_command.extend(command)
        return ssh_command

    def run_command_in_background(self, command, cwd=None):
        ssh_command = self._prepare_ssh_command(command)
        return subprocess.Popen(ssh_command)

    def _get_options(self):
        options = [
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "LogLevel=error",
        ]
        if self.private_key:
            options.extend(["-i", self.private_key])
        return options

    def copy_file(self, local_file_path, remote_file_path):
        scp_command = ["scp", "-P", self.port]
        scp_command.extend(self._get_options())
        scp_command.extend(
            [local_file_path, "{}@{}:{}".format(self.user, self.ip, remote_file_path)]
        )
        subprocess.check_call(scp_command)


class QemuTestbed(SshTestbed):
    def __init__(self, image_path, ssh_port, user, private_key, redirect_ports):
        super().__init__("localhost", ssh_port, user, private_key)
        self._image_path = image_path
        self._process = None
        self._redirect_ports = redirect_ports

    def create(self):
        logger.info("Running the snappy image in a virtual machine.")
        ports = " -redir tcp:{}::22".format(self.port)
        for port in self._redirect_ports:
            ports += " -redir tcp:{0}::{0}".format(port)
        qemu_command = (
            "qemu-system-{}"
            + " -snapshot"
            + " -enable-kvm"
            + " -m 512 -nographic -net user -net nic,model=virtio"
            + " -drive file={}"
            + ",if=virtio"
            + " -monitor none -serial none"
        ).format(platform.machine(), self._image_path) + ports
        self._process = subprocess.Popen(qemu_command, shell=True)

    def delete(self):
        if self._process:
            self._process.kill()
