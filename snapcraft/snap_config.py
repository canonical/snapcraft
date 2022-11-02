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

"""Snap config file definitions and helpers."""
from typing import Any, Dict, Literal, Optional

import pydantic
from craft_cli import emit
from snaphelpers import SnapConfigOptions, SnapCtlError

from snapcraft.utils import is_snapcraft_running_from_snap


class SnapConfig(pydantic.BaseModel, extra=pydantic.Extra.forbid):
    """Data stored in a snap config.

    :param provider: provider to use. Valid values are 'lxd' and 'multipass'.
    """

    provider: Optional[Literal["lxd", "multipass"]] = None

    @pydantic.validator("provider", pre=True)
    @classmethod
    def convert_to_lower(cls, provider):
        """Convert provider value to lowercase."""
        return provider.lower()

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "SnapConfig":
        """Create and populate a new ``SnapConfig`` object from dictionary data.

        The unmarshal method validates entries in the input dictionary, populating
        the corresponding fields in the data object.

        :param data: The dictionary data to unmarshal.

        :return: The newly created object.

        :raise TypeError: If data is not a dictionary.
        :raise ValueError: If data is invalid.
        """
        if not isinstance(data, dict):
            raise TypeError("snap config data is not a dictionary")

        try:
            snap_config = cls(**data)
        except pydantic.ValidationError as error:
            # TODO: use `_format_pydantic_errors()` from projects.py
            raise ValueError(f"error parsing snap config: {error}") from error

        return snap_config


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
    except (AttributeError, SnapCtlError) as error:
        # snaphelpers raises an error (either AttributeError or SnapCtlError) when
        # it fails to get the snap config. this can occur when running inside a
        # docker or podman container where snapd is not available
        emit.debug("Could not retrieve the snap config. Is snapd running?")
        emit.trace(f"snaphelpers error: {error!r}")
        return None

    snap_config.fetch()

    emit.debug(f"Retrieved snap config: {snap_config.as_dict()}")

    return SnapConfig.unmarshal(snap_config.as_dict())
