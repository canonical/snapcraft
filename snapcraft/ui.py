#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

"""User interface for Snapcraft."""

from craft_cli.messages import Emitter, EmitterMode

import snapcraft
from snapcraft import utils
from snapcraft.errors import SnapcraftError

emit = Emitter()


def init(argv) -> None:
    """Initialize the Command Line Interface from argv."""
    emitter_mode = EmitterMode.NORMAL

    try:
        if len(argv) < 2 or argv[1] != "pack":
            raise SnapcraftError("Only pack is supported.")

        if len(argv) == 3 and argv[2] == "--verbose":
            emitter_mode = EmitterMode.VERBOSE
        elif len(argv) == 3 and argv[2] != "--verbose":
            raise SnapcraftError("Only option supported for pack is '--verbose'.")
    finally:
        args = {
            "mode": emitter_mode,
            "appname": "snapcraft",
            "greeting": f"Snapcraft version {snapcraft.__version__}.",
        }

        if utils.is_managed_mode():
            args["log_filepath"] = utils.get_managed_environment_log_path()

        emit.init(**args)  # type: ignore
