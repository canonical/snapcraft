# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""The Snapcraft Python plugin.."""

import logging
from typing import Optional

from craft_parts.plugins import python_plugin
from overrides import override

logger = logging.getLogger(__name__)


_RUNTIME_PYTHON_PATH = {
    ("core22", "strict"): "/usr/bin/python3.10",
    ("core22", "devmode"): "/usr/bin/python3.10",
    ("core22", "classic"): None,  # classic snaps must provision python
}


class PythonPlugin(python_plugin.PythonPlugin):
    """A Python plugin for Snapcraft.

    This plugin extends Craft-parts' vanilla Python plugin to properly
    set the Python interpreter according to the Snapcraft base and
    confinement parameters.
    """

    @override
    def _get_system_python_interpreter(self) -> Optional[str]:
        base = self._part_info.project_base
        confinement = self._part_info.confinement
        interpreter = _RUNTIME_PYTHON_PATH.get((base, confinement))
        logger.debug(
            "Using python interpreter '%s' for base '%s', confinement '%s'",
            interpreter,
            base,
            confinement,
        )
        return interpreter
