# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Snapcraft Service Factory."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from craft_application import ServiceFactory

from snapcraft import models

# Add new services to this mapping to add them to the service factory
# Internal service name : Stringified service class name
_SERVICES: dict[str, str] = {
    "init": "Init",
    "provider": "Provider",
    "lifecycle": "Lifecycle",
    "package": "Package",
    "remote_build": "RemoteBuild",
    "confdb_schemas": "ConfdbSchemas",
}


@dataclass
class SnapcraftServiceFactory(ServiceFactory):
    """Snapcraft-specific Service Factory."""

    project: models.Project | None = None  # type: ignore[reportIncompatibleVariableOverride]

    if TYPE_CHECKING:
        from services import ConfdbSchemas

        # Allow static type check to report correct types for Snapcraft services
        confdb_schemas: ConfdbSchemas = None  # type: ignore[assignment]


def register_snapcraft_services() -> None:
    """Register Snapcraft-specific services."""
    for name, service in _SERVICES.items():
        module_name = name.replace("_", "")
        SnapcraftServiceFactory.register(
            name, service, module=f"snapcraft.services.{module_name}"
        )
