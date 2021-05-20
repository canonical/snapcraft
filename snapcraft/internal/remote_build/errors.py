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
from snapcraft.internal.errors import SnapcraftException


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


class GitNotFoundVersionError(RemoteBuildBaseError):

    fmt = (
        "This remote-build project requires `git` to be installed "
        "because of it use of `version: git`.  Either install `git`, "
        "set the `version` statically, or use the `snapcraftctl set-version` "
        "part scriptlet with `adopt-info`."
    )


class GitNotFoundProviderError(RemoteBuildBaseError):

    fmt = (
        "The remote build provider ({provider!r}) requires "
        "the use of the `git` utility, and `git` is not installed. "
        "Please install `git` and run the command again."
    )

    def __init__(self, *, provider: str) -> None:
        super().__init__(provider=provider)


class BaseRequiredError(RemoteBuildBaseError):

    fmt = (
        "Remote build currently requires that the project uses bases.\n"
        "Please specify an appropriate `base` keyword and try again."
    )


class RemoteBuilderNotSupportedError(RemoteBuildBaseError):

    fmt = (
        "Remote builder {provider!r} is not supported, please choose "
        "a different one and try again."
    )

    def __init__(self, *, provider: str) -> None:
        super().__init__(provider=provider)


class RemoteBuildTimeoutError(RemoteBuildBaseError):

    fmt = "Remote build exceeded configured timeout."


class RemoteBuilderError(RemoteBuildBaseError):

    fmt = "Remote builder failed with error: {builder_error!r}"

    def __init__(self, *, builder_error: str) -> None:
        super().__init__(builder_error=builder_error)


class UnsupportedArchitectureError(RemoteBuildBaseError):

    fmt = (
        "The following architectures are not supported by the remote builder: "
        "{archs}.\nPlease remove them from the architecture list "
        "and try again."
    )

    def __init__(self, *, archs: List[str]) -> None:
        super().__init__(archs=", ".join(archs))


class UnsupportedVersionScriptError(RemoteBuildBaseError):

    fmt = (
        "Remote-build does not support the use of `version-script`.\n"
        "Please use `snapcraftctl set-version` part scriptlet with "
        "`adopt-info`, or set `version` statically."
    )


class AcceptPublicUploadError(RemoteBuildBaseError):

    fmt = (
        "Remote build needs explicit acknowledgement that data sent to build servers "
        "is public.\n"
        "In non-interactive runs, please use option --launchpad-accept-public-upload."
    )


class LaunchpadGitPushError(SnapcraftException):
    def __init__(self, *, command: str, exit_code: int) -> None:
        self._command = command
        self._exit_code = exit_code

    def get_brief(self) -> str:
        return "Failed to push sources to Launchpad."

    def get_details(self) -> str:
        return f"Command {self._command!r} failed with exit code {self._exit_code!r}."

    def get_resolution(self) -> str:
        return "Verify connectivity to https://git.launchpad.net and retry build."


class LaunchpadHttpsError(SnapcraftException):
    def get_brief(self) -> str:
        return "Failed to connect to Launchpad API service."

    def get_resolution(self) -> str:
        return "Verify connectivity to https://api.launchpad.net and retry build."
