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
import os
from pathlib import Path
from typing import Any, Dict, Literal, Optional

import pydantic
import yaml
from craft_cli import emit


class SnapConfig(pydantic.BaseModel, extra=pydantic.Extra.forbid):
    """Data stored in a snap config.

    :param provider: provider to use. Valid values are 'lxd' and 'multipass'.
    """

    provider: Optional[Literal["lxd", "multipass"]]

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
            raise ValueError("error loading snap-config.yaml") from error

        return snap_config

    @classmethod
    def load(cls) -> Optional["SnapConfig"]:
        """Get snap configuration data from $SNAP_DATA/snap-config.yaml."""
        emit.debug("Loading snap-config.yaml")

        snap_data = os.getenv("SNAP_DATA")
        if not snap_data:
            emit.debug(
                "Not loading snap-config.yaml because the environmental "
                "variable '$SNAP_DATA' isn't defined."
            )
            return None

        config_file = Path(snap_data) / "snap-config.yaml"
        if not config_file.exists():
            emit.debug("Not loading {config_file!r} because the file doesn't exist.")
            return None

        with open(config_file, encoding="utf-8") as file:
            config_data = yaml.safe_load(file)

        emit.debug("Loaded snap-config.yaml")
        return cls.unmarshal(config_data)
