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
import os
import sys
from pathlib import Path
from typing import Optional, Sequence

from snapcraft import legacy, lifecycle, ui
from snapcraft.errors import SnapcraftError
from snapcraft.projects import Project, load_project


def run(argv: Optional[Sequence] = None):
    """Run the CLI."""
    if argv is None:
        argv = sys.argv

    # check legacy managed environment
    if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
        print("On a legacy managed environment, executing legacy snapcraft")
        legacy.run()
        return

    # TODO: add proper command parsing using craft-cli.
    #       for now it runs the new code if snapcraft is invoked without arguments
    #       or with a step name.
    if len(argv) > 1 and argv[1] not in ["pull", "build", "stage", "prime"]:
        # legacy passthrough
        print("Not a step command, executing legacy snapcraft")
        legacy.run()
        return

    if not load_and_process_project(argv):
        legacy.run()


def load_and_process_project(argv: Sequence):
    # load project and verify base
    project: Optional[Project] = None
    for project_file in [
        "snapcraft.yaml",
        "snap/snapcraft.yaml",
        "build-aux/snap/snapcraft.yaml",
        ".snapcraft.yaml",
    ]:
        if Path(project_file).is_file():
            try:
                project = load_project(project_file)
            except SnapcraftError:
                # legacy passthrough
                print("Error loading project, executing legacy snapcraft")
                return False

    if not project:
        # legacy passthrough
        print("Couldn't find project yaml, executing legacy snapcraft")
        return False

    if project.base != "core22":
        # legacy passthrough
        print("Base not core22, executing legacy snapcraft")
        return False

    # set lib loggers to debug level so that all messages are sent to Emitter
    for lib_name in ("craft_providers", "craft_parts"):
        logger = logging.getLogger(lib_name)
        logger.setLevel(logging.DEBUG)

    try:
        ui.init(argv)
        lifecycle.run(project)
    except SnapcraftError as error:
        ui.emit.error(error)
        sys.exit(1)
    finally:
        ui.emit.ended_ok()

    return True
