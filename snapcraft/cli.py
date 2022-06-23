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

import contextlib
import logging
import os
import sys

import craft_cli
import craft_store
from craft_cli import ArgumentParsingError, EmitterMode, ProvideHelpException, emit

import snapcraft
import snapcraft_legacy
from snapcraft import __version__, errors, utils
from snapcraft_legacy.cli import legacy

from . import commands
from .commands import store
from .legacy_cli import _LIB_NAMES, _ORIGINAL_LIB_NAME_LOG_LEVEL, run_legacy

COMMAND_GROUPS = [
    craft_cli.CommandGroup(
        "Lifecycle",
        [
            commands.CleanCommand,
            commands.PullCommand,
            commands.BuildCommand,
            commands.StageCommand,
            commands.PrimeCommand,
            commands.PackCommand,
            commands.SnapCommand,  # hidden (legacy compatibility)
            commands.StoreLegacyRemoteBuildCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Extensions",
        [
            commands.ListExtensionsCommand,
            commands.ExtensionsCommand,  # hidden (alias to list-extensions)
            commands.ExpandExtensionsCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Account",
        [
            commands.StoreLoginCommand,
            commands.StoreExportLoginCommand,
            commands.StoreLogoutCommand,
            commands.StoreWhoAmICommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Snap Names",
        [
            commands.StoreRegisterCommand,
            commands.StoreNamesCommand,
            commands.StoreLegacyListRegisteredCommand,
            commands.StoreLegacyListCommand,
            commands.StoreLegacyMetricsCommand,
            commands.StoreLegacyUploadMetadataCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Snap Release Management",
        [
            commands.StoreReleaseCommand,
            commands.StoreCloseCommand,
            commands.StoreStatusCommand,
            commands.StoreUploadCommand,
            commands.StoreLegacyPushCommand,  # hidden (legacy for upload)
            commands.StoreLegacyPromoteCommand,
            commands.StoreLegacyListRevisionsCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Snap Tracks",
        [
            commands.StoreListTracksCommand,
            commands.StoreTracksCommand,  # hidden (alias to list-tracks)
            commands.StoreLegacySetDefaultTrackCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Assertions",
        [
            commands.StoreLegacyCreateKeyCommand,
            commands.StoreLegacyEditValidationSetsCommand,
            commands.StoreLegacyGatedCommand,
            commands.StoreLegacyListValidationSetsCommand,
            commands.StoreLegacyRegisterKeyCommand,
            commands.StoreLegacySignBuildCommand,
            commands.StoreLegacyValidateCommand,
            commands.StoreLegacyListKeysCommand,
        ],
    ),
    craft_cli.CommandGroup("Other", [commands.VersionCommand]),
]

GLOBAL_ARGS = [
    craft_cli.GlobalArgument(
        "version", "flag", "-V", "--version", "Show the application version and exit"
    )
]


def get_dispatcher() -> craft_cli.Dispatcher:
    """Return an instance of Dispatcher.

    Run all the checks and setup required to ensure the Dispatcher can run.
    """
    # Run the legacy implementation if inside a legacy managed environment.
    if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
        snapcraft.BasePlugin = snapcraft_legacy.BasePlugin  # type: ignore
        snapcraft.ProjectOptions = snapcraft_legacy.ProjectOptions  # type: ignore
        legacy.legacy_run()

    # set lib loggers to debug level so that all messages are sent to Emitter
    for lib_name in _LIB_NAMES:
        logger = logging.getLogger(lib_name)
        _ORIGINAL_LIB_NAME_LOG_LEVEL[lib_name] = logger.level
        logger.setLevel(logging.DEBUG)

    if utils.is_managed_mode():
        log_filepath = utils.get_managed_environment_log_path()
    else:
        log_filepath = None

    emit.init(
        mode=EmitterMode.NORMAL,
        appname="snapcraft",
        greeting=f"Starting Snapcraft {__version__}",
        log_filepath=log_filepath,
    )

    return craft_cli.Dispatcher(
        "snapcraft",
        COMMAND_GROUPS,
        summary="Package, distribute, and update snaps for Linux and IoT",
        extra_global_args=GLOBAL_ARGS,
        default_command=commands.PackCommand,
    )


def run():
    """Run the CLI."""
    dispatcher = get_dispatcher()
    try:
        global_args = dispatcher.pre_parse_args(sys.argv[1:])
        if global_args.get("version"):
            emit.message(f"snapcraft {__version__}")
        else:
            dispatcher.load_command(None)
            dispatcher.run()
        emit.ended_ok()
        retcode = 0
    except ArgumentParsingError as err:
        # TODO https://github.com/canonical/craft-cli/issues/78
        with contextlib.suppress(KeyError, IndexError):
            if (
                err.__context__ is not None
                and err.__context__.args[0]  # pylint: disable=no-member
                not in dispatcher.commands
            ):
                run_legacy(err)
        print(err, file=sys.stderr)  # to stderr, as argparse normally does
        emit.ended_ok()
        retcode = 1
    except ProvideHelpException as err:
        print(err, file=sys.stderr)  # to stderr, as argparse normally does
        emit.ended_ok()
        retcode = 0
    except errors.LegacyFallback as err:
        run_legacy(err)
    except craft_store.errors.NoKeyringError as err:
        emit.error(
            craft_cli.errors.CraftError(
                f"craft-store error: {err}",
                resolution=(
                    "Ensure the keyring is working or "
                    f"{store.constants.ENVIRONMENT_STORE_CREDENTIALS} "
                    "is correctly exported into the environment"
                ),
                docs_url="https://snapcraft.io/docs/snapcraft-authentication",
            )
        )
        retcode = 1
    except craft_store.errors.CraftStoreError as err:
        emit.error(craft_cli.errors.CraftError(f"craft-store error: {err}"))
        retcode = 1
    except errors.SnapcraftError as err:
        emit.error(err)
        retcode = 1

    return retcode
