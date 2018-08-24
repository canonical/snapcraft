#!/usr/bin/python3
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

import os
import logging
import subprocess
from typing import Sequence
from typing import Callable, Union  # noqa: F401

from snapcraft.internal.build_providers import errors as _provider_errors

logger = logging.getLogger(name=__name__)


def _run(command: Sequence[str]) -> None:
    logger.debug("Running {}".format(" ".join(command)))
    subprocess.check_call(command)


def _run_output(command: Sequence[str]) -> bytes:
    logger.debug("Running {}".format(" ".join(command)))
    return subprocess.check_output(command)


class LXDInstanceProvider:
    """Provide the minimal Provider methods to be able to use the SnapInjector.
    """

    _SNAPS_MOUNTPOINT = os.path.join("/var", "cache", "snapcraft", "snaps")

    def __init__(self, instance_name: str) -> None:
        self._instance_name = instance_name

    def run(self, command: Sequence[str], hide_output: bool = False):
        cmd = ["lxc", "exec", self._instance_name, "--"] + list(command)
        if hide_output:
            runnable = (
                _run_output
            )  # type: Callable[[Sequence[str]], Union[bytes, None]]
        else:
            runnable = _run
        try:
            runnable(cmd)
        except subprocess.CalledProcessError as process_error:
            raise _provider_errors.ProviderExecError(
                provider_name="lxd", command=command, exit_code=process_error.returncode
            ) from process_error

    def mount_snaps_directory(self) -> None:
        cmd = [
            "lxc",
            "config",
            "device",
            "add",
            self._instance_name,
            "snapd-snaps",
            "disk",
            "source={}".format(),
            "path={}".format(self._SNAPS_MOUNTPOINT),
        ]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise _provider_errors.ProviderMountError(
                provider_name="lxd", exit_code=process_error.returncode
            ) from process_error

    def unmount_snaps_directory(self) -> None:
        cmd = ["lxc", "config", "device", "remove", self._instance_name, "snapd-snaps"]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise _provider_errors.ProviderUnMountError(
                provider_name="lxd", exit_code=process_error.returncode
            ) from process_error

    def push_file(self, source: str, destination: str):
        cmd = [
            "lxc",
            "file",
            "push",
            source,
            "{}{}".format(self._instance_name, destination),
        ]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise _provider_errors.ProviderFileCopyError(
                provider_name="lxd", exit_code=process_error.returncode
            ) from process_error

    def pull_file(self, source: str, destination: str):
        cmd = [
            "lxc",
            "file",
            "pull",
            "{}{}".format(self._instance_name, source),
            destination,
        ]
        try:
            _run(cmd)
        except subprocess.CalledProcessError as process_error:
            raise _provider_errors.ProviderFileCopyError(
                provider_name="lxd", exit_code=process_error.returncode
            ) from process_error
