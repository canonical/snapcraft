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

"""Snapcraft provided plugin registration."""

import craft_parts
from craft_parts.plugins.plugins import PluginType

from .colcon_plugin import ColconPlugin
from .conda_plugin import CondaPlugin
from .flutter_plugin import FlutterPlugin
from .kernel_plugin import KernelPlugin
from .python_plugin import PythonPlugin


def get_plugins(core22: bool) -> dict[str, PluginType]:
    """Get the dict of Snapcraft-specific plugins.

    :param core22: Whether core22-only plugins should be included.
    """
    plugins = {
        "colcon": ColconPlugin,
        "conda": CondaPlugin,
        "flutter": FlutterPlugin,
        "python": PythonPlugin,
    }

    if core22:
        plugins["kernel"] = KernelPlugin

    return plugins


def register() -> None:
    """Register Snapcraft plugins for core22."""
    craft_parts.plugins.register(get_plugins(core22=True))
