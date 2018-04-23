# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2018 Canonical Ltd
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
from typing import List  # noqa

from snapcraft.internal.errors import SnapcraftError


logger = logging.getLogger(__name__)


class ContainerError(SnapcraftError):
    """Base class for all container-related exceptions.

    :cvar fmt: A format string that daughter classes override

    """


class ContainerArchitectureError(ContainerError):

    fmt = (
        "Failed to detect container architecture: "
        "The output from 'lxc info' could not be read:\n{lxc_info}"
    )

    def __init__(self, lxc_info: str) -> None:
        super().__init__(lxc_info=lxc_info)


class ContainerLXDNotInstalledError(ContainerError):

    fmt = (
        "Failed to initialize container: "
        "LXD could not be found. "
        "You must have LXD installed in order to use cleanbuild.\n"
        "Refer to the documentation at "
        "https://linuxcontainers.org/lxd/getting-started-cli."
    )

    def __init__(self):
        super().__init__()


class ContainerLXDRemoteNotFoundError(ContainerError):

    fmt = (
        "Failed to initialize container: "
        "There are either no permissions or the remote {remote!r} "
        "does not exist. "
        "Verify the existing remotes by running `lxc remote list`.\n"
        "Refer to the documentation at "
        "https://linuxcontainers.org/lxd/getting-started-cli."
    )

    def __init__(self, remote):
        super().__init__(remote=remote)


class ContainerLXDSetupError(ContainerError):

    fmt = (
        "Failed to initialize container: "
        "Something seems to be wrong with your installation of LXD.\n"
        "Refer to the documentation at "
        "https://linuxcontainers.org/lxd/getting-started-cli."
    )

    def __init__(self):
        super().__init__()


class ContainerStartFailedError(ContainerError):

    fmt = (
        "Failed to start container: "
        "The local folder could not be mounted into the container. "
        "The files /etc/subuid and /etc/subgid need to "
        "contain this line for mounting the local folder:\n"
        "    root:1000:1\n"
        "Note: Add the line to both files, do not "
        "remove any existing lines.\n"
        "Restart LXD after making this change.\n"
    )

    def __init__(self):
        super().__init__()


class ContainerCreationFailedError(ContainerError):

    fmt = (
        "Failed to create container: "
        "A new LXD container could not be created.\n"
        "Refer to the documentation at "
        "https://linuxcontainers.org/lxd/getting-started-cli."
    )

    def __init__(self):
        super().__init__()


class ContainerNetworkError(ContainerError):

    fmt = (
        "Failed to get a network connection in the container: "
        "Could not successfully ping {url!r}. "
        "If using a proxy, check its configuration.\n"
        "Refer to the documentation at "
        "https://linuxcontainers.org/lxd/getting-started-cli."
    )

    def __init__(self, url):
        super().__init__(url=url)


class ContainerRunError(ContainerError):

    fmt = (
        'The following command failed to run: '
        '{command!r} exited with {exit_code}\n'
    )

    def __init__(self, *, command, exit_code):
        if isinstance(command, list):
            command = ' '.join(command)
        super().__init__(command=command, exit_code=exit_code)


class ContainerSnapcraftCmdError(ContainerRunError):

    fmt = (
        'Snapcraft command failed in the container: '
        '{command!r} exited with {exit_code}\n'
    )

    def __init__(self, *, command, exit_code):
        if isinstance(command, list):
            command = ' '.join(command)
        super().__init__(command=command, exit_code=exit_code)


class SnapdQueryError(ContainerError):

    fmt = (
        "Failed to get snap in container: "
        "Error querying {snap!r} snap: {message}"
    )

    def __init__(self, snap, message):
        super().__init__(snap=snap, message=message)


class SnapdConnectionError(ContainerError):

    fmt = (
        "Failed to get snap in container: "
        "Could not connect to {api!r}."
    )

    def __init__(self, api):
        super().__init__(api=api)
