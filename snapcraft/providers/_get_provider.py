# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd.
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

from ._lxd import LXDProvider
from ._multipass import MultipassProvider
from ._provider import Provider


def get_provider() -> Provider:
    """Get the configured or appropriate provider for the host OS.

    If platform is not Linux, use Multipass.

    If platform is Linux:
    (1) use provider specified with SNAPCRAFT_PROVIDER if running
        in developer mode,
    (2) use provider specified with snap configuration if running
        as snap,
    (3) default to platform default (LXD on Linux).

    :return: Provider instance.
    """
    provider = os.getenv("SNAPCRAFT_PROVIDER")

    if provider is None:
        provider = _get_platform_default_provider()

    if provider == "lxd":
        return LXDProvider()

    if provider == "multipass":
        return MultipassProvider()

    raise RuntimeError(f"Unsupported provider specified: {provider!r}.")


def _get_platform_default_provider() -> str:
    if sys.platform == "linux":
        return "lxd"

    return "multipass"
