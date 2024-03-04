# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Extension commands for core24 (forwarded to the shared core22 implementation)."""

from craft_application.commands import AppCommand

from snapcraft.commands import core22


class ExpandExtensions(AppCommand, core22.ExpandExtensionsCommand):
    """core24 command to expand extensions."""


class ListExtensions(AppCommand, core22.ListExtensionsCommand):
    """core24 command to list extensions."""
