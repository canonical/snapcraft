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

"""Snapcraft services."""

from .assertions import Assertion
from .buildplan import BuildPlan
from .init import Init
from .lifecycle import Lifecycle
from .package import Package
from .project import Project
from .provider import Provider
from .confdbschemas import ConfdbSchemas
from .service_factory import (
    SnapcraftServiceFactory,
    register_snapcraft_services,
)

__all__ = [
    "Assertion",
    "BuildPlan",
    "ConfdbSchemas",
    "Init",
    "Lifecycle",
    "Package",
    "Project",
    "Provider",
    "register_snapcraft_services",
    "SnapcraftServiceFactory",
]
