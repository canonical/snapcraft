# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2024 Canonical Ltd.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License version 3, as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
#  SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Service class for assertions."""
from __future__ import annotations

from typing import TYPE_CHECKING

from craft_application.services import base

if TYPE_CHECKING:  # pragma: no cover
    from craft_application import AppMetadata, ServiceFactory


class AssertionService(base.AppService):
    """Abstract service for interacting with assertions."""

    def __init__(self, app: AppMetadata, services: ServiceFactory) -> None:
        super().__init__(app=app, services=services)
