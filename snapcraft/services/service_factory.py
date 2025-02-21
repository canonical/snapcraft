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

from craft_application import AppService, ServiceFactory

from snapcraft import models, services

# Add new services to this mapping to add them to the service factory
_SERVICES: dict[str, type[AppService]] = {
    "init": services.Init,
    "provider": services.Provider,
    "lifecycle": services.Lifecycle,
    "package": services.Package,
    "remote_build": services.RemoteBuild,
    "confdbs": services.Confdbs,
}


@dataclass
class SnapcraftServiceFactory(ServiceFactory):
    """Snapcraft-specific Service Factory."""

    project: models.Project | None = None  # type: ignore[reportIncompatibleVariableOverride]

    if TYPE_CHECKING:
        # Allow static type check to report correct types for Snapcraft services
        confdbs: services.Confdbs = None  # type: ignore[assignment]

    @classmethod
    def register_snapcraft_plugins(cls) -> None:
        """Register Snapcraft-specific services."""
        for name, service in _SERVICES.items():
            cls.register(name, service)
