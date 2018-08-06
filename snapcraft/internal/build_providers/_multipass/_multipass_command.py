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
import signal
import shutil
import subprocess
from typing import Any, Callable, List, Union  # noqa: F401

from snapcraft.internal.build_providers import errors


logger = logging.getLogger(__name__)


def _run(command: List) -> None:
    logger.debug("Running {}".format(" ".join(command)))
    subprocess.check_call(command)


def _run_output(command: List) -> bytes:
    logger.debug("Running {}".format(" ".join(command)))
    return subprocess.check_output(command)


def _ignore_signal(sig, stack):
    # We only do this for SIGUSR1 as it is the one we mostly
    # care about.
    if sig == signal.SIGUSR1:
        sig = "SIGUSR1"
    logger.debug("Ignoring {!r}: {!r}".format(sig, dir(stack)))


class MultipassCommand:
    """An object representation of common multipass cli commands."""

    provider_name = "multipass"

    def __init__(self) -> None:
        """Initialize a MultipassCommand instance.

        :raises errors.ProviderCommandNotFound:
            if the multipass command is not found.
        """
        provider_cmd = "multipass"
        if not shutil.which(provider_cmd):
            raise errors.ProviderCommandNotFound(command=provider_cmd)
        self.provider_cmd = provider_cmd
        # Workaround for https://github.com/CanonicalLtd/multipass/issues/221
        signal.signal(signal.SIGUSR1, _ignore_signal)

    def launch(self, *, instance_name: str, image: str, remote: str = None) -> None:
        """Passthrough for running multipass launch.

        :param str instance_name: the name the launched instance will have.
        :param str image: the image to create the instance with.
        :param str remote: the remote server to retrieve the image from.
        """
        if remote is not None:
            image = "{}:{}".format(remote, image)
        cmd = [self.provider_cmd, "launch", image, "--name", instance_name]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderLaunchError(
                provider_name=self.provider_name, exit_code=process_error.returncode
            ) from process_error

    def stop(self, *, instance_name: str) -> None:
        """Passthrough for running multipass stop.

        :param str instance_name: the name of the instance to stop.
        """
        cmd = [self.provider_cmd, "stop", instance_name]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderStopError(
                provider_name=self.provider_name, exit_code=process_error.returncode
            ) from process_error

    def delete(self, *, instance_name: str, purge=True) -> None:
        """Passthrough for running multipass delete.

        :param str instance_name: the name of the instance to delete.
        :param bool purge: if true, purge the instance's image after deleting.
        """
        cmd = [self.provider_cmd, "delete", instance_name]
        if purge:
            cmd.append("--purge")
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderDeleteError(
                provider_name=self.provider_name, exit_code=process_error.returncode
            ) from process_error

    def execute(
        self, *, command: List[str], instance_name: str, hide_output: bool = False
    ) -> None:
        """Passthrough for running multipass exec.

        :param list command: the command to exectute on the instance.
        :param str instance_name: the name of the instance to execute command.
        """
        cmd = [self.provider_cmd, "exec", instance_name, "--"] + command
        if hide_output:
            runnable = _run_output  # type: Callable[[List[Any]], Union[bytes, None]]
        else:
            runnable = _run
        try:
            runnable(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderExecError(
                provider_name=self.provider_name,
                command=command,
                exit_code=process_error.returncode,
            ) from process_error

    def mount(self, *, source: str, target: str) -> None:
        """Passthrough for running multipass mount.

        :param str source: path to the local directory to mount.
        :param str target: mountpoint inside the instance in the form of
                           <instance-name>:path.
        :raises errors.ProviderMountError: when the mount operation fails.
        """
        cmd = [self.provider_cmd, "mount", source, target]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderMountError(
                provider_name=self.provider_name, exit_code=process_error.returncode
            ) from process_error

    def copy_files(self, *, source: str, destination: str) -> None:
        """Passthrough for running multipass copy-files.

        :param str source: the source file to copy, using syntax expected
                           by multipass.
        :param str destination: the destination of the copied file, using
                                syntax expected by multipass.
        """
        cmd = [self.provider_cmd, "copy-files", source, destination]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderFileCopyError(
                provider_name=self.provider_name, exit_code=process_error.returncode
            ) from process_error

    def info(self, *, instance_name: str, output_format: str = None) -> bytes:
        """Passthrough for running multipass info."""
        cmd = [self.provider_cmd, "info", instance_name]
        if output_format is not None:
            cmd.extend(["--format", output_format])
        try:
            return _run_output(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderInfoError(
                provider_name=self.provider_name, exit_code=process_error.returncode
            ) from process_error
