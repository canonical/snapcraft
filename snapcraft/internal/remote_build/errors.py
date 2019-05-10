# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from typing import List, Sequence  # noqa: F401

from snapcraft.internal.errors import SnapcraftError as _SnapcraftError


class RemoteBuildBaseError(_SnapcraftError):
    """Base Exception for all remote build related exceptions."""


class RemoteBuildNotFoundError(RemoteBuildBaseError):

    fmt = (
        "Remote build not found, please make sure project {name!r} and build "
        "number {req_number} are correct. If this is an old build, it may "
        "have expired."
    )

    def __init__(self, *, name: str, req_number: int) -> None:
        super().__init__(name=name, req_number=req_number)


class NoLaunchpadUsernameError(RemoteBuildBaseError):

    fmt = "Please set your Launchpad username using the '--user' option."


class NotGitRepositoryError(RemoteBuildBaseError):

    fmt = "Current directory is not a git repository."


class GitNotFoundError(RemoteBuildBaseError):

    fmt = (
        "This snapcraft version requires git to be installed on your system to "
        "execute remote-build. Please install git and run the command again."
    )


class InvalidVersionGitError(RemoteBuildBaseError):

    fmt = (
        "Can't use version 'git' in a non-git remote build.\n"
        "Either build with --git, or use a different version in snapcraft.yaml."
    )


class RemoteBuilderNotSupportedError(RemoteBuildBaseError):

    fmt = (
        "Remote builder {provider!r} is not supported, please choose "
        "a different one and try again."
    )

    def __init__(self, *, provider: str) -> None:
        super().__init__(provider=provider)


class RemoteBuilderNotReadyError(RemoteBuildBaseError):

    fmt = "Remote builder is not ready, please wait a few moments and try again."


class UnsupportedArchitectureError(RemoteBuildBaseError):

    fmt = (
        "The following architectures are not supported by the remote builder: "
        "{archs}.\nPlease remove them from the architecture list "
        "and try again."
    )

    def __init__(self, *, archs: List[str]) -> None:
        super().__init__(archs=", ".join(archs))
