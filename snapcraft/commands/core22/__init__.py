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

"""Snapcraft commands for core22 base."""

from .account import (
    StoreExportLoginCommand,
    StoreLoginCommand,
    StoreLogoutCommand,
    StoreWhoAmICommand,
)
from .discovery import ListPluginsCommand, PluginsCommand
from .extensions import (
    ExpandExtensionsCommand,
    ExtensionsCommand,
    ListExtensionsCommand,
)
from .init import InitCommand
from .lifecycle import (
    BuildCommand,
    CleanCommand,
    PackCommand,
    PrimeCommand,
    PullCommand,
    SnapCommand,
    StageCommand,
    TryCommand,
)
from .lint import LintCommand
from .names import StoreNamesCommand, StoreRegisterCommand
from .remote import RemoteBuildCommand
from .validation_sets import StoreEditValidationSetsCommand

__all__ = [
    "BuildCommand",
    "CleanCommand",
    "ExpandExtensionsCommand",
    "ExtensionsCommand",
    "InitCommand",
    "LintCommand",
    "ListExtensionsCommand",
    "ListPluginsCommand",
    "PackCommand",
    "PluginsCommand",
    "PrimeCommand",
    "PullCommand",
    "RemoteBuildCommand",
    "SnapCommand",
    "StageCommand",
    "StoreEditValidationSetsCommand",
    "StoreExportLoginCommand",
    "StoreLoginCommand",
    "StoreLogoutCommand",
    "StoreNamesCommand",
    "StoreRegisterCommand",
    "StoreWhoAmICommand",
    "TryCommand",
]
