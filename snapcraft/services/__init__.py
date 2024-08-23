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

from .assertions import AssertionService
from .lifecycle import Lifecycle
from .package import Package
from .provider import Provider
from .registry_sets import RegistrySetsService
from .remotebuild import RemoteBuild
from .service_factory import SnapcraftServiceFactory

__all__ = [
    "AssertionService",
    "Lifecycle",
    "Package",
    "Provider",
    "RegistrySetsService",
    "RemoteBuild",
    "SnapcraftServiceFactory",
]
