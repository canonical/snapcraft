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

import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import sys
import warnings

_VALID_V1_PLUGINS = [
    "ant",
    "catkin-tools",
    "conda",
    "dump",
    "gradle",
    "make",
    "nil",
    "python",
    "rust",
    "autotools",
    "cmake",
    "go",
    "kbuild",
    "maven",
    "nodejs",
    "qmake",
    "scons",
    "catkin",
    "colcon",
    "crystal",
    "dotnet",
    "godeps",
    "kernel",
    "meson",
    "plainbox-provider",
    "ruby",
    "waf",
]


class LegacyPluginLoader(importlib.abc.Loader):
    @classmethod
    def create_module(cls, spec):
        # Load the plugin from the new location.
        plugin_name = spec.name.split(".")[-1]
        return importlib.import_module(f"snapcraft.plugins.v1.{plugin_name}")

    @classmethod
    def exec_module(cls, module):
        # Rewrite the module __name__ to have that of the legacy import path.
        plugin_name = module.__name__.split(".")[-1]
        module.__name__ = f"snapcraft.plugins.{plugin_name}"
        return module


class LegacyPluginPathFinder(importlib.machinery.PathFinder):
    @classmethod
    def find_spec(cls, fullname, path=None, target=None):
        # Ensure plugins using their original import paths can be found and
        # warn about their new import path.
        if fullname in [f"snapcraft.plugins.{p}" for p in _VALID_V1_PLUGINS]:
            warnings.warn("Plugin import path has changed to 'snapcraft.plugins.v1'")
            return importlib.machinery.ModuleSpec(fullname, LegacyPluginLoader)
        else:
            return None


# https://docs.python.org/3/library/importlib.html#importlib.machinery.PathFinder
sys.meta_path.append(LegacyPluginPathFinder)  # type: ignore
