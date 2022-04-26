# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Snap Store constants."""

from typing import Final

ENVIRONMENT_STORE_CREDENTIALS: Final[str] = "SNAPCRAFT_STORE_CREDENTIALS"
"""Environment variable where credentials can be picked up from."""

ENVIRONMENT_STORE_AUTH: Final[str] = "SNAPCRAFT_STORE_AUTH"
"""Environment variable used to set an alterntive login method.

The only setting that changes the behavior is `candid`, every
other value uses Ubuntu SSO.
"""

STORE_URL: Final[str] = "https://dashboard.snapcraft.io"
"""Default store backend URL."""

STORE_UPLOAD_URL: Final[str] = "https://storage.snapcraftcontent.com"
"""Default store upload URL."""

UBUNTU_ONE_SSO_URL = "https://login.ubuntu.com"
"""Default Ubuntu One Login URL."""

DEFAULT_SERIES = "16"
"""Legacy value for older generation Snap Store APIs."""
