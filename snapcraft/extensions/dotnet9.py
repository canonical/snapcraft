# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
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

from typing_extensions import override

from .dotnet_base import DotnetExtensionBase


class Dotnet9Extension(DotnetExtensionBase):
    """
    An extension that eases the creation of snaps that integrate with
    the .NET 9 content snaps.
    """

    @property
    @override
    def runtime_content_snap_name(self) -> str:
        return "dotnet-runtime-90"

    @property
    @override
    def versioned_plugin_name(self) -> str:
        return "dotnet9"
