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

from ._plugin import PluginV1  # noqa: F401

# The plugin code requires imports that are platform specific.
if sys.platform == "linux":
    from .ant import AntPlugin  # noqa: F401
    from .autotools import AutotoolsPlugin  # noqa: F401
    from .catkin import CatkinPlugin  # noqa: F401
    from .catkin_tools import CatkinToolsPlugin  # noqa: F401
    from .cmake import CMakePlugin  # noqa: F401
    from .colcon import ColconPlugin  # noqa: F401
    from .conda import CondaPlugin  # noqa: F401
    from .crystal import CrystalPlugin  # noqa: F401
    from .dotnet import DotNetPlugin  # noqa: F401
    from .dump import DumpPlugin  # noqa: F401
    from .flutter import FlutterPlugin  # noqa: F401
    from .go import GoPlugin  # noqa: F401
    from .godeps import GodepsPlugin  # noqa: F401
    from .gradle import GradlePlugin  # noqa: F401
    from .kbuild import KBuildPlugin  # noqa: F401
    from .kernel import KernelPlugin  # noqa: F401
    from .make import MakePlugin  # noqa: F401
    from .maven import MavenPlugin  # noqa: F401
    from .meson import MesonPlugin  # noqa: F401
    from .nil import NilPlugin  # noqa: F401
    from .nodejs import NodePlugin  # noqa: F401
    from .plainbox_provider import PlainboxProviderPlugin  # noqa: F401
    from .python import PythonPlugin  # noqa: F401
    from .qmake import QmakePlugin  # noqa: F401
    from .ruby import RubyPlugin  # noqa: F401
    from .rust import RustPlugin  # noqa: F401
    from .scons import SconsPlugin  # noqa: F401
    from .waf import WafPlugin  # noqa: F401
