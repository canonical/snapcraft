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

from ._plugin import PluginV2  # noqa: F401

# The plugin code requires imports that are platform specific.
if sys.platform == "linux":
    from .autotools import AutotoolsPlugin  # noqa: F401
    from .cmake import CMakePlugin  # noqa: F401
    from .colcon import ColconPlugin  # noqa: F401
    from .dump import DumpPlugin  # noqa: F401
    from .go import GoPlugin  # noqa: F401
    from .make import MakePlugin  # noqa: F401
    from .meson import MesonPlugin  # noqa: F401
    from .nil import NilPlugin  # noqa: F401
    from .npm import NpmPlugin  # noqa: F401
    from .python import PythonPlugin  # noqa: F401
    from .rust import RustPlugin  # noqa: F401
