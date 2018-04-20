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


class ProviderNotSupported(_SnapcraftError):

    fmt = (
        'The {provider!r} provider is not supported, please choose a '
        'different one and try again.'
    )

    def __init__(self, *, provider: str) -> None:
        super().__init__(provider=provider)


class ProviderCommandNotFound(_SnapcraftError):

    fmt = (
        'The {command!r} command is necessary to be able to build in this '
        'environment.\n'
        'Install {command!r} or if already installed, ensure it is '
        'on the system PATH, and try again.'
    )

    def __init__(self, *, command: str) -> None:
        super().__init__(command=command)


class ProviderLaunchError(_SnapcraftError):

    fmt = (
        'An error occurred when trying to launch the instance with '
        '{provider_name!r}.'
    )

    def __init__(self, *, provider_name: str) -> None:
        super().__init__(provider_name=provider_name)


class ProviderStopError(_SnapcraftError):

    fmt = (
        'An error occurred when trying to stop the instance with '
        '{provider_name!r}.'
    )

    def __init__(self, *, provider_name: str) -> None:
        super().__init__(provider_name=provider_name)


class ProviderDeleteError(_SnapcraftError):

    fmt = (
        'An error occurred when trying to delete the instance with '
        '{provider_name!r}.'
    )

    def __init__(self, *, provider_name: str) -> None:
        super().__init__(provider_name=provider_name)


class ProviderExecError(_SnapcraftError):

    fmt = (
        'An error occurred when trying to execute {command_string!r} with '
        '{provider_name!r}.'
    )

    def __init__(self, *, provider_name: str, command: Sequence[str]) -> None:
        command_string = ' '.join(shlex.quote(i) for i in command)
        super().__init__(provider_name=provider_name, command=command,
                         command_string=command_string)


class ProviderFileCopyError(_SnapcraftError):

    fmt = (
        'An error occurred when trying to copy files using {provider_name!r}.'
    )

    def __init__(self, *, provider_name: str) -> None:
        super().__init__(provider_name=provider_name)


class ProviderInfoError(_SnapcraftError):

    fmt = (
        'An error occurred when using {provider_name!r} to '
        'query the status of the instance.'
    )

    def __init__(self, *, provider_name: str) -> None:
        super().__init__(provider_name=provider_name)


class ProviderInfoDataKeyError(_SnapcraftError):

    fmt = (
        'The data returned by {provider_name!r} was not expected. '
        'It is missing some key data {key_info!r} in {data!r}.'
    )

    def __init__(self, *, provider_name: str, key_info: str,
                 data: Dict[str, Any]) -> None:
        super().__init__(provider_name=provider_name, key_info=key_info,
                         data=data)


class ProviderBadDataError(_SnapcraftError):

    fmt = (
        'The data returned by {provider_name!r} was not expected '
        'or in the wrong format: {data!r}.'
    )

    def __init__(self, *, provider_name: str, data: str) -> None:
        super().__init__(provider_name=provider_name, data=data)
