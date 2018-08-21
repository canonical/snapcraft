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

import shlex
from typing import Any, Dict
from typing import Sequence  # noqa: F401

from snapcraft.internal.errors import SnapcraftError as _SnapcraftError


class ProviderNotSupportedError(_SnapcraftError):

    fmt = (
        "The {provider!r} provider is not supported, please choose a "
        "different one and try again."
    )

    def __init__(self, *, provider: str) -> None:
        super().__init__(provider=provider)


class ProviderCommandNotFound(_SnapcraftError):

    fmt = (
        "{command!r} command not found: this command is necessary to build in "
        "this environment.\n"
        "Install {command!r} or if already installed, ensure it is "
        "on the system PATH, and try again."
    )

    def __init__(self, *, command: str) -> None:
        super().__init__(command=command)


class _GenericProviderError(_SnapcraftError):

    fmt = (
        "An error occurred when trying to {action} the instance with "
        "{provider_name!r}: returned exit code {exit_code!r}.\n"
        "Ensure that {provider_name!r} is setup correctly and try again."
    )


class ProviderCommunicationError(_SnapcraftError):

    fmt = (
        "An error occurred when trying to communicate with the instance "
        "using {protocol!r} over port {port}: {error}."
    )

    def __init__(self, *, protocol: str, port: int, error: str) -> None:
        super().__init__(protocol=protocol, port=port, error=error)


class ProviderLaunchError(_GenericProviderError):
    def __init__(self, *, provider_name: str, exit_code: int) -> None:
        super().__init__(
            action="launch", provider_name=provider_name, exit_code=exit_code
        )


class ProviderStopError(_GenericProviderError):
    def __init__(self, *, provider_name: str, exit_code: int) -> None:
        super().__init__(
            action="stop", provider_name=provider_name, exit_code=exit_code
        )


class ProviderDeleteError(_GenericProviderError):
    def __init__(self, *, provider_name: str, exit_code: int) -> None:
        super().__init__(
            action="delete", provider_name=provider_name, exit_code=exit_code
        )


class ProviderExecError(_SnapcraftError):

    fmt = (
        "An error occurred when trying to execute {command_string!r} with "
        "{provider_name!r}: returned exit code {exit_code!r}."
    )

    def __init__(
        self, *, provider_name: str, command: Sequence[str], exit_code: int
    ) -> None:
        command_string = " ".join(shlex.quote(i) for i in command)
        super().__init__(
            provider_name=provider_name,
            command=command,
            command_string=command_string,
            exit_code=exit_code,
        )


class ProviderMountError(_SnapcraftError):

    fmt = (
        "An error occurred when trying to mount using {provider_name!r}: "
        "returned exit code {exit_code!r}."
    )

    def __init__(self, *, provider_name: str, exit_code: int) -> None:
        super().__init__(provider_name=provider_name, exit_code=exit_code)


class ProviderUnMountError(_SnapcraftError):

    fmt = (
        "An error occurred when trying to unmount using {provider_name!r}: "
        "returned exit code {exit_code!r}."
    )

    def __init__(self, *, provider_name: str, exit_code: int) -> None:
        super().__init__(provider_name=provider_name, exit_code=exit_code)


class ProviderFileCopyError(_SnapcraftError):

    fmt = (
        "An error occurred when trying to copy files using {provider_name!r}: "
        "returned exit code {exit_code!r}."
    )

    def __init__(self, *, provider_name: str, exit_code: int) -> None:
        super().__init__(provider_name=provider_name, exit_code=exit_code)


class ProviderInfoError(_SnapcraftError):

    fmt = (
        "An error occurred when using {provider_name!r} to "
        "query the status of the instance: returned exit code {exit_code!r}."
    )

    def __init__(self, *, provider_name: str, exit_code: int) -> None:
        super().__init__(provider_name=provider_name, exit_code=exit_code)


class ProviderInfoDataKeyError(_SnapcraftError):

    fmt = (
        "The data returned by {provider_name!r} was not expected. "
        "It is missing a required key {missing_key!r} in {data!r}."
    )

    def __init__(
        self, *, provider_name: str, missing_key: str, data: Dict[str, Any]
    ) -> None:
        super().__init__(
            provider_name=provider_name, missing_key=missing_key, data=data
        )


class ProviderBadDataError(_SnapcraftError):

    fmt = (
        "The data returned by {provider_name!r} was not expected "
        "or in the wrong format: {data!r}."
    )

    def __init__(self, *, provider_name: str, data: str) -> None:
        super().__init__(provider_name=provider_name, data=data)


class SSHKeyFileNotFoundError(_SnapcraftError):

    fmt = (
        "{private_key_file_path!r} does not exist. "
        "A private key is required.\n"
        "Please file a report on https://launchpad.net/snapcraft/+filebug"
    )

    def __init__(self, *, private_key_file_path: str) -> None:
        super().__init__(private_key_file_path=private_key_file_path)


class BuildImageRequestError(_SnapcraftError):

    fmt = (
        "Failed to retrieve build image for {base!r}: "
        "The server responded with HTTP status code {status_code!r}.\n"
        "Contact the creator of {base!r} for assistance if the issue persists."
    )

    def __init__(self, *, base: str, status_code: int) -> None:
        super().__init__(base=base, status_code=status_code)


class BuildImageSetupError(_SnapcraftError):

    fmt = (
        "Failed to set up the build image for this project: "
        "The command exited with exit code {exit_code!r}."
    )

    def __init__(self, *, exit_code: int) -> None:
        super().__init__(exit_code=exit_code)


class BuildImageForBaseMissing(_SnapcraftError):
    fmt = (
        "Cannot find suitable build image for base {base!r} and architecture {snap_arch!r}.\n"
        "Contact the creator of {base!r} for assistance."
    )

    def __init__(self, *, base: str, snap_arch: str) -> None:
        super().__init__(base=base, snap_arch=snap_arch)


class BuildImageChecksumError(_SnapcraftError):
    fmt = (
        "Expected the {algorithm!r} calculated digest for the build image to be {expected!r}, "
        "but it was {calculated!r}.\n"
        "Please verify there are no network issues and try again."
    )

    def __init__(self, *, expected: str, calculated: str, algorithm: str) -> None:
        super().__init__(expected=expected, calculated=calculated, algorithm=algorithm)
