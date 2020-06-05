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
from typing import Dict, Type, Union, TYPE_CHECKING

from snapcraft.internal import errors
from . import v1, v2

PluginTypes = Union[Type[v1.PluginV1], Type[v2.PluginV2]]

# TODO: segregate into base specific plugins.
if sys.platform == "linux" or TYPE_CHECKING:
    _PLUGINS: Dict[str, Dict[str, PluginTypes]] = {
        "legacy": {
            "ant": v1.AntPlugin,
            "autotools": v1.AutotoolsPlugin,
            "catkin": v1.CatkinPlugin,
            "catkin-tools": v1.CatkinToolsPlugin,
            "cmake": v1.CMakePlugin,
            "colcon": v1.ColconPlugin,
            "conda": v1.CondaPlugin,
            "crystal": v1.CrystalPlugin,
            "dotnet": v1.DotNetPlugin,
            "dump": v1.DumpPlugin,
            "flutter": v1.FlutterPlugin,
            "go": v1.GoPlugin,
            "godeps": v1.GodepsPlugin,
            "gradle": v1.GradlePlugin,
            "kbuild": v1.KBuildPlugin,
            "kernel": v1.KernelPlugin,
            "make": v1.MakePlugin,
            "maven": v1.MavenPlugin,
            "meson": v1.MesonPlugin,
            "nil": v1.NilPlugin,
            "nodejs": v1.NodePlugin,
            "plainbox-provider": v1.PlainboxProviderPlugin,
            "python": v1.PythonPlugin,
            "qmake": v1.QmakePlugin,
            "ruby": v1.RubyPlugin,
            "rust": v1.RustPlugin,
            "scons": v1.SconsPlugin,
            "waf": v1.WafPlugin,
        },
        "core20": {
            "autotools": v2.AutotoolsPlugin,
            "cmake": v2.CMakePlugin,
            "colcon": v2.ColconPlugin,
            "dump": v2.DumpPlugin,
            "go": v2.GoPlugin,
            "make": v2.MakePlugin,
            "meson": v2.MesonPlugin,
            "nil": v2.NilPlugin,
            "npm": v2.NpmPlugin,
            "python": v2.PythonPlugin,
            "rust": v2.RustPlugin,
        },
    }
else:
    # We cannot import the plugins on anything but linux.
    _PLUGINS = {"legacy": {}}


def get_plugin_for_base(plugin_name: str, *, build_base: str) -> PluginTypes:
    """
    Return a suitable plugin implementation for build_base.

    :raises errors.PluginError: when a plugin implementation cannot be found
                                for build_base.
    """
    # TODO: segregate this more and remove the checks UnsupportedBase checks
    # from the plugins
    if build_base in ("core", "core16", "core18"):
        build_base = "legacy"
    try:
        plugin_classes = _PLUGINS[build_base]
        return plugin_classes[plugin_name]
    except KeyError:
        raise errors.PluginError(f"unknown plugin: {plugin_name!r}")
