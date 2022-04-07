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

import logging
import os
import sys

import craft_cli
from craft_cli import ArgumentParsingError, EmitterMode, emit

from snapcraft import __version__, errors, utils
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
            commands.PackCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Extensions",
        [
            commands.ListExtensionsCommand,
            # hidden command, alias to list-extensions.
            commands.ExtensionsCommand,
            commands.ExpandExtensionsCommand,
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
    # Run the legacy implementation if inside a legacy managed environment.
    if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
        legacy.legacy_run()

    # Let legacy snapcraft handle --help until we have all command stubs registered
    # in craft-cli.
    if "-h" in sys.argv or "--help" in sys.argv:
        legacy.legacy_run()

    # set lib loggers to debug level so that all messages are sent to Emitter
    for lib_name in ("craft_parts", "craft_providers"):
        logger = logging.getLogger(lib_name)
        logger.setLevel(logging.DEBUG)

    emit_args = {
        "mode": EmitterMode.NORMAL,
        "appname": "snapcraft",
        "greeting": f"Starting Snapcraft {__version__}",
    }

    if utils.is_managed_mode():
        emit_args["log_filepath"] = utils.get_managed_environment_log_path()

    emit.init(**emit_args)

    dispatcher = craft_cli.Dispatcher(
        "snapcraft",
        COMMAND_GROUPS,
        summary="What's the app about",
        extra_global_args=GLOBAL_ARGS,
        default_command=commands.PackCommand,
    )

    try:
        global_args = dispatcher.pre_parse_args(sys.argv[1:])
        if global_args.get("version"):
            emit.message(f"snapcraft {__version__}")
        else:
            dispatcher.load_command(None)
            dispatcher.run()
        emit.ended_ok()
    except (errors.LegacyFallback, ArgumentParsingError) as err:
        emit.trace(f"run legacy implementation: {err!s}")
        emit.ended_ok()
        legacy.legacy_run()
    except errors.SnapcraftError as err:
        emit.error(err)
        sys.exit(1)
