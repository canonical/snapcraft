# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021-2022 Canonical Ltd.
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

"""Build environment provider support for snapcraft."""

import os
import sys
from typing import Optional

from craft_cli import emit

from snapcraft.snap_config import get_snap_config

from ._lxd import LXDProvider
from ._multipass import MultipassProvider
from ._provider import Provider


def get_provider(provider: Optional[str] = None) -> Provider:
    """Get the configured or appropriate provider for the host OS.

    To determine the appropriate provider,
    (1) use provider specified in the function argument,
    (2) get the provider from the environment,
    (3) use provider specified with snap configuration,
    (4) default to platform default (LXD on Linux, otherwise Multipass).

    :return: Provider instance.
    """
    chosen_provider = ""
    env_provider = os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT")

    # load snap config file
    snap_config = get_snap_config()
    snap_provider = snap_config.provider if snap_config else None

    # (1) use provider specified in the function argument
    if provider:
        emit.debug(f"Using provider {provider!r} passed as an argument.")
        chosen_provider = provider

    # (2) get the provider from the environment
    elif env_provider:
        emit.debug(
            f"Using provider {env_provider!r} from environmental "
            "variable 'SNAPCRAFT_BUILD_ENVIRONMENT'."
        )
        chosen_provider = env_provider

    # (3) use provider specified with snap configuration
    elif snap_provider:
        emit.debug(f"Using provider {snap_provider!r} from snap config.")
        chosen_provider = snap_provider

    # (4) default to platform default (LXD on Linux, otherwise Multipass)
    elif sys.platform == "linux":
        emit.debug("Using default provider 'lxd' on linux system.")
        chosen_provider = "lxd"
    else:
        emit.debug("Using default provider 'multipass' on non-linux system.")
        chosen_provider = "multipass"

    # ignore case
    chosen_provider = chosen_provider.lower()

    # return the chosen provider
    if chosen_provider == "lxd":
        return LXDProvider()
    if chosen_provider == "multipass":
        return MultipassProvider()

    raise ValueError(f"unsupported provider specified: {chosen_provider!r}")
