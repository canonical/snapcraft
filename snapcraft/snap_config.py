# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Snap config file definitions and helpers."""

from typing import Annotated, Literal, Optional

import craft_application.models
import pydantic
from craft_cli import emit
from snaphelpers import SnapConfigOptions, SnapCtlError

from snapcraft.utils import is_snapcraft_running_from_snap

ProviderName = Annotated[
    Literal["lxd", "multipass"], pydantic.BeforeValidator(lambda name: name.lower())
]


class SnapConfig(craft_application.models.CraftBaseModel):
    """Data stored in a snap config.

    :param provider: provider to use. Valid values are 'lxd' and 'multipass'.
    """

    provider: ProviderName | None = None


def get_snap_config() -> Optional[SnapConfig]:
    """Get validated snap configuration.

    :return: SnapConfig. If not running as a snap, return None.
    """
    if not is_snapcraft_running_from_snap():
        emit.debug(
            "Not loading snap config because snapcraft is not running as a snap."
        )
        return None

    try:
        snap_config = SnapConfigOptions(keys=["provider"])
        # even if the initialization of SnapConfigOptions succeeds, `fetch()` may
        # raise the same errors since it makes calls to snapd
        snap_config.fetch()
    except (AttributeError, SnapCtlError) as error:
        # snaphelpers raises an error (either AttributeError or SnapCtlError) when
        # it fails to get the snap config. this can occur when running inside a
        # docker or podman container where snapd is not available
        emit.debug("Could not retrieve the snap config. Is snapd running?")
        emit.trace(f"snaphelpers error: {error!r}")
        return None

    emit.debug(f"Retrieved snap config: {snap_config.as_dict()}")

    return SnapConfig.unmarshal(snap_config.as_dict())
