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

import os
from typing import Annotated, Any, Literal

import pydantic
from craft_cli import emit
from snaphelpers import SnapConfigOptions, SnapCtlError
from typing_extensions import Self


def _normalize_provider(name: Any) -> Any:
    # Non-strings are left for pydantic to reject.
    if isinstance(name, str):
        return name.lower()
    return name


def _alias_generator(name: str) -> str:
    """Match CraftBaseModel YAML keys, which replace underscores with hyphens."""
    return name.replace("_", "-")


ProviderName = Annotated[
    Literal["lxd", "multipass"],
    pydantic.BeforeValidator(_normalize_provider),
]


# Importing CraftBaseModel loads craft-parts and craft-providers, which can make
# the configure hook exceed snapd's timeout.
class SnapConfig(pydantic.BaseModel):
    """Data stored in a snap config.

    :param provider: provider to use. Valid values are 'lxd' and 'multipass'.
    """

    model_config = pydantic.ConfigDict(
        validate_assignment=True,
        extra="forbid",
        populate_by_name=True,
        alias_generator=_alias_generator,
        coerce_numbers_to_str=True,
    )

    provider: ProviderName | None = None

    @classmethod
    def unmarshal(cls, data: dict[str, Any]) -> Self:
        """Create and validate a SnapConfig from snapd config data.

        :param data: The dictionary data to unmarshal.
        :raises TypeError: If data is not a dictionary.
        :raises pydantic.ValidationError: If data does not match the model.
        """
        if not isinstance(data, dict):
            raise TypeError("Project data is not a dictionary")

        return cls.model_validate(data)


# Not imported from snapcraft.utils: that import loads craft-parts.
def is_snapcraft_running_from_snap() -> bool:
    """Check if snapcraft is running from the snap."""
    return os.getenv("SNAP_NAME") == "snapcraft" and os.getenv("SNAP") is not None


def get_snap_config() -> SnapConfig | None:
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
