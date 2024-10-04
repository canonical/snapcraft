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

"""The Snapcraft Poetry plugin."""

from craft_parts.plugins import poetry_plugin
from overrides import override

from snapcraft.parts.plugins import python_common


class PoetryPlugin(poetry_plugin.PoetryPlugin):
    """A Poetry plugin for Snapcraft."""

    @override
    def _get_system_python_interpreter(self) -> str | None:
        return python_common.get_system_interpreter(self._part_info)
