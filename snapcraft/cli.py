# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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

import argparse
import contextlib
import os
import sys
from dataclasses import dataclass
from typing import Any, Dict

import craft_application.commands
import craft_cli
import craft_store
from craft_application.errors import RemoteBuildError
from craft_application.util import strtobool
from craft_cli import ArgumentParsingError, EmitterMode, ProvideHelpException, emit
from craft_providers import ProviderError

from snapcraft import errors, store, utils
from snapcraft.parts import plugins

from . import commands
from .legacy_cli import run_legacy


@dataclass
class CommandGroup:
    """Dataclass to hold a command group."""

    name: str
    commands: list


CORE22_LIFECYCLE_COMMAND_GROUP = CommandGroup(
    "Lifecycle",
    [
        commands.core22.CleanCommand,
        commands.core22.PullCommand,
        commands.core22.BuildCommand,
        commands.core22.StageCommand,
        commands.core22.PrimeCommand,
        commands.core22.PackCommand,
        commands.core22.SnapCommand,  # hidden (legacy compatibility)
        commands.core22.TryCommand,
    ],
)

CORE24_LIFECYCLE_COMMAND_GROUP = CommandGroup(
    "Lifecycle",
    [
        craft_application.commands.lifecycle.CleanCommand,
        craft_application.commands.lifecycle.PullCommand,
        craft_application.commands.lifecycle.BuildCommand,
        craft_application.commands.lifecycle.StageCommand,
        craft_application.commands.lifecycle.PrimeCommand,
        commands.PackCommand,
        commands.SnapCommand,  # Hidden (legacy compatibility)
        commands.RemoteBuildCommand,
        commands.TryCommand,
    ],
)

COMMAND_GROUPS = [
    CommandGroup(
        "Plugins",
        [
            commands.PluginsCommand,
            commands.ListPluginsCommand,
        ],
    ),
    CommandGroup(
        "Extensions",
        [
            commands.ListExtensionsCommand,
            commands.ExtensionsCommand,  # hidden (alias to list-extensions)
            commands.ExpandExtensionsCommand,
        ],
    ),
    CommandGroup(
        "Store Account",
        [
            commands.StoreLoginCommand,
            commands.StoreExportLoginCommand,
            commands.StoreLogoutCommand,
            commands.StoreWhoAmICommand,
        ],
    ),
    CommandGroup(
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
    CommandGroup(
        "Store Snap Release Management",
        [
            commands.StoreReleaseCommand,
            commands.StoreCloseCommand,
            commands.StoreStatusCommand,
            commands.StoreUploadCommand,
            commands.StoreLegacyPushCommand,  # hidden (legacy for upload)
            commands.StoreLegacyPromoteCommand,
            commands.StoreListRevisionsCommand,
            commands.StoreRevisionsCommand,  # hidden (alias to list-revisions)
        ],
    ),
    CommandGroup(
        "Store Snap Tracks",
        [
            commands.StoreListTracksCommand,
            commands.StoreTracksCommand,  # hidden (alias to list-tracks)
            commands.StoreLegacySetDefaultTrackCommand,
        ],
    ),
    CommandGroup(
        "Store Key Management",
        [
            commands.StoreLegacyCreateKeyCommand,
            commands.StoreLegacyRegisterKeyCommand,
            commands.StoreLegacySignBuildCommand,
            commands.StoreLegacyListKeysCommand,
        ],
    ),
    CommandGroup(
        "Store Validation Sets",
        [
            commands.StoreEditValidationSetsCommand,
            commands.StoreLegacyListValidationSetsCommand,
            commands.StoreLegacyValidateCommand,
            commands.StoreLegacyGatedCommand,
        ],
    ),
    CommandGroup(
        "Other",
        [
            *craft_application.commands.get_other_command_group().commands,
            commands.LintCommand,
            commands.InitCommand,
        ],
    ),
]

GLOBAL_ARGS = [
    craft_cli.GlobalArgument(
        "version", "flag", "-V", "--version", "Show the application version and exit"
    ),
    craft_cli.GlobalArgument("trace", "flag", "-t", "--trace", argparse.SUPPRESS),
]


def get_verbosity() -> EmitterMode:
    """Return the verbosity level to use.

    if SNAPCRAFT_ENABLE_DEVELOPER_DEBUG is set, the
    default verbosity will be set to EmitterMode.DEBUG.

    If stdin is closed, the default verbosity will be
    set to EmitterMode.VERBOSE.
    """
    verbosity = EmitterMode.BRIEF

    if not sys.stdin.isatty():
        verbosity = EmitterMode.VERBOSE

    with contextlib.suppress(ValueError):
        # Parse environment variable for backwards compatibility with launchpad
        if strtobool(os.getenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", "n").strip()):
            verbosity = EmitterMode.DEBUG

    # if defined, use environmental variable SNAPCRAFT_VERBOSITY_LEVEL
    verbosity_env = os.getenv("SNAPCRAFT_VERBOSITY_LEVEL")
    if verbosity_env:
        try:
            verbosity = EmitterMode[verbosity_env.strip().upper()]
        except KeyError:
            values = utils.humanize_list(
                [e.name.lower() for e in EmitterMode], "and", sort=False
            )
            raise ArgumentParsingError(
                f"cannot parse verbosity level {verbosity_env!r} from environment "
                f"variable SNAPCRAFT_VERBOSITY_LEVEL (valid values are {values})"
            ) from KeyError

    return verbosity


def get_dispatcher() -> craft_cli.Dispatcher:
    """Return an instance of Dispatcher."""
    craft_cli_command_groups = [
        craft_cli.CommandGroup(group.name, group.commands)
        for group in COMMAND_GROUPS + [CORE22_LIFECYCLE_COMMAND_GROUP]
    ]

    return craft_cli.Dispatcher(
        "snapcraft",
        craft_cli_command_groups,
        summary="Package, distribute, and update snaps for Linux and IoT",
        extra_global_args=GLOBAL_ARGS,
        default_command=commands.core22.PackCommand,
    )


def _run_dispatcher(
    dispatcher: craft_cli.Dispatcher, global_args: Dict[str, Any]
) -> None:
    if global_args.get("trace"):
        emit.message(
            "Options -t and --trace are deprecated, use --verbosity=debug instead."
        )
        emit.set_mode(EmitterMode.DEBUG)

    dispatcher.load_command(None)
    dispatcher.run()
    emit.ended_ok()


def _emit_error(error, cause=None):
    """Emit the error in a centralized way so we can alter it consistently."""
    # set the cause, if any
    if cause is not None:
        error.__cause__ = cause

    # Do not report the internal logpath if running inside instance
    if utils.is_managed_mode():
        error.logpath_report = False

    emit.error(error)


def run():  # noqa: C901 (complex-structure)
    """Run the CLI."""
    dispatcher = get_dispatcher()
    retcode = 1

    try:
        # Register our own plugins
        global_args = dispatcher.pre_parse_args(sys.argv[1:])
        plugins.register()

        _run_dispatcher(dispatcher, global_args)
        retcode = 0
    except ArgumentParsingError as err:
        # TODO https://github.com/canonical/craft-cli/issues/78
        with contextlib.suppress(KeyError, IndexError):
            if (
                err.__context__ is not None
                and err.__context__.args[0] not in dispatcher.commands
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
    except KeyboardInterrupt as err:
        _emit_error(craft_cli.errors.CraftError("Interrupted."), cause=err)
        retcode = 1
    except craft_store.errors.NoKeyringError as err:
        _emit_error(
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
        _emit_error(craft_cli.errors.CraftError(f"craft-store error: {err}"))
        retcode = 1
    except ProviderError as err:
        _emit_error(craft_cli.errors.CraftError(f"craft-providers error: {err}"))
        retcode = 1
    except errors.LinterError as err:
        emit.error(craft_cli.errors.CraftError(f"linter error: {err}"))
        retcode = err.exit_code
    except RemoteBuildError as err:
        emit.error(
            craft_cli.errors.CraftError(
                message=f"remote-build error: {err}",
                docs_url="https://snapcraft.io/docs/remote-build",
            )
        )
        retcode = 1
    except errors.SnapcraftError as err:
        _emit_error(err)
        retcode = 1

    return retcode
