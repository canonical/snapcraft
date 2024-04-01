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

"""Snapcraft commands."""

from . import core22, legacy
from .extensions import ExpandExtensions, ListExtensions
from .lifecycle import SnapCommand
from .manage import StoreCloseCommand, StoreReleaseCommand
from .remote import RemoteBuildCommand
from .status import (
    StoreListRevisionsCommand,
    StoreListTracksCommand,
    StoreRevisionsCommand,
    StoreStatusCommand,
    StoreTracksCommand,
)
from .upload import StoreUploadCommand

__all__ = [
    "core22",
    "legacy",
    "SnapCommand",
    "RemoteBuildCommand",
    "ExpandExtensions",
    "ListExtensions",
    "StoreUploadCommand",
    "StoreListRevisionsCommand",
    "StoreListTracksCommand",
    "StoreRevisionsCommand",
    "StoreStatusCommand",
    "StoreTracksCommand",
    "StoreCloseCommand",
    "StoreReleaseCommand",
]
