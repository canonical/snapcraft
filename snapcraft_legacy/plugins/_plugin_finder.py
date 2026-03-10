# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import sys
from typing import TYPE_CHECKING, Dict, Type

from snapcraft_legacy.internal import errors

from . import v2

PluginTypes = Type[v2.PluginV2]

if sys.platform == "linux" or TYPE_CHECKING:
    _PLUGINS: Dict[str, PluginTypes] = {
        "autotools": v2.AutotoolsPlugin,
        "catkin": v2.CatkinPlugin,
        "catkin-tools": v2.CatkinToolsPlugin,
        "cmake": v2.CMakePlugin,
        "colcon": v2.ColconPlugin,
        "conda": v2.CondaPlugin,
        "crystal": v2.CrystalPlugin,
        "dump": v2.DumpPlugin,
        "go": v2.GoPlugin,
        "kernel": v2.KernelPlugin,
        "make": v2.MakePlugin,
        "meson": v2.MesonPlugin,
        "nil": v2.NilPlugin,
        "npm": v2.NpmPlugin,
        "python": v2.PythonPlugin,
        "qmake": v2.QMakePlugin,
        "rust": v2.RustPlugin,
    }
else:
    # We cannot import the plugins on anything but linux.
    _PLUGINS = {}


def get_plugin_for_base(plugin_name: str, *, build_base: str) -> PluginTypes:
    """
    Return a suitable plugin implementation for build_base.

    :raises errors.PluginError: when a plugin implementation cannot be found
                                for build_base.
    """
    # TODO: segregate this more and remove the checks UnsupportedBase checks
    # from the plugins

    # Default for unknown, and core20
    if build_base in ("core18", "core"):
        raise errors.PluginBaseError(plugin_name=plugin_name, base=build_base)
    elif build_base == "":
        # This should never happen when using build_base from Project.
        raise RuntimeError("'build_base' cannot be unset.")

    try:
        return _PLUGINS[plugin_name]
    except KeyError:
        raise errors.PluginError(f"unknown plugin: {plugin_name!r}")
