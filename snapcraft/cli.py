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

"""Command-line application entry point."""

import sys

import craft_cli
from craft_cli import EmitterMode, emit

from snapcraft import __version__, errors
from snapcraft_legacy.cli import legacy

from . import commands

COMMAND_GROUPS = [
    craft_cli.CommandGroup(
        "Lifecycle",
        [
            commands.PullCommand,
            commands.BuildCommand,
            commands.StageCommand,
            commands.PrimeCommand,
        ],
    ),
    craft_cli.CommandGroup("Other", [commands.VersionCommand]),
]

GLOBAL_ARGS = [
    craft_cli.GlobalArgument(
        "version", "flag", "-V", "--version", "Show the application version and exit"
    )
]


def run():
    """Run the CLI."""
    # Let legacy snapcraft handle --help until we have all command stubs registered
    # in craft-cli.
    if "-h" in sys.argv or "--help" in sys.argv:
        legacy.legacy_run()

    emit.init(EmitterMode.NORMAL, "snapcraft", f"Starting Snapcraft {__version__}")
    dispatcher = craft_cli.Dispatcher(
        "snapcraft",
        COMMAND_GROUPS,
        summary="What's the app about",
        extra_global_args=GLOBAL_ARGS,
    )

    try:
        global_args = dispatcher.pre_parse_args(sys.argv[1:])
        if global_args.get("version"):
            emit.message(f"snapcraft {__version__}")
        else:
            dispatcher.load_command(None)
            dispatcher.run()
        emit.ended_ok()
    except errors.SnapcraftError as err:
        emit.error(err)
    except craft_cli.ArgumentParsingError:
        emit.ended_ok()
        legacy.legacy_run()
