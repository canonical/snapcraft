# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd.
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

"""Temporary CLI implementation."""

import logging
import sys
from typing import Optional, Sequence

from craft_cli.errors import CraftError

from . import lifecycle, ui


def run(argv: Optional[Sequence] = None):
    """Run the CLI."""
    if argv is None:
        argv = sys.argv

    # set lib loggers to debug level so that all messages are sent to Emitter
    for lib_name in ("craft_providers", "craft_parts"):
        logger = logging.getLogger(lib_name)
        logger.setLevel(logging.DEBUG)

    try:
        ui.init(argv)
        lifecycle.pack()
    except CraftError as error:
        ui.emit.error(error)
        sys.exit(1)
    finally:
        ui.emit.ended_ok()
