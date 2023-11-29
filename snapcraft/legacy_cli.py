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

"""Handle re-execution into legacy code."""

import logging
from typing import Dict, Optional

from craft_cli import emit

import snapcraft
import snapcraft_legacy
from snapcraft_legacy.cli import legacy

_LIB_NAMES = ("craft_parts", "craft_providers", "craft_store", "snapcraft.remote")
_ORIGINAL_LIB_NAME_LOG_LEVEL: Dict[str, int] = {}


def run_legacy(err: Optional[Exception] = None):
    """Run legacy implementation."""
    # Reset the libraries to their original log level
    for lib_name in _LIB_NAMES:
        logger = logging.getLogger(lib_name)
        logger.setLevel(_ORIGINAL_LIB_NAME_LOG_LEVEL[lib_name])

    snapcraft.ProjectOptions = snapcraft_legacy.ProjectOptions  # type: ignore

    # Legacy does not use craft-cli
    if err is not None:
        emit.trace(f"run legacy implementation: {err!s}")
    emit.ended_ok()

    legacy.legacy_run()
