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
from typing import Any

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

CORE22_LIFECYCLE_COMMAND_GROUP = craft_cli.CommandGroup(
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

CORE24_LIFECYCLE_COMMAND_GROUP = craft_cli.CommandGroup(
    "Lifecycle",
    [
        craft_application.commands.lifecycle.CleanCommand,
        craft_application.commands.lifecycle.PullCommand,
        craft_application.commands.lifecycle.BuildCommand,
        craft_application.commands.lifecycle.StageCommand,
        craft_application.commands.lifecycle.PrimeCommand,
        craft_application.commands.lifecycle.TestCommand,
        commands.PackCommand,
        commands.SnapCommand,  # Hidden (legacy compatibility)
        commands.RemoteBuildCommand,
        commands.TryCommand,
    ],
)

COMMAND_GROUPS = [
    craft_cli.CommandGroup(
        "Plugins",
        [
            commands.PluginsCommand,
            commands.ListPluginsCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Extensions",
        [
            commands.ListExtensionsCommand,  # hidden (alias to extensions)
            commands.ExtensionsCommand,
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
            commands.StoreListRevisionsCommand,  # hidden (alias to revisions)
            commands.StoreRevisionsCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Snap Tracks",
        [
            commands.StoreListTracksCommand,  # hidden (alias to tracks)
            commands.StoreTracksCommand,
            commands.StoreLegacySetDefaultTrackCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Key Management",
        [
            commands.StoreLegacyCreateKeyCommand,
            commands.StoreLegacyRegisterKeyCommand,
            commands.StoreLegacySignBuildCommand,
            commands.StoreLegacyListKeysCommand,
            commands.StoreLegacyKeysCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Validation Sets",
        [
            commands.StoreEditValidationSetsCommand,
            commands.StoreListValidationSetsCommand,  # hidden (alias to validation-sets)
            commands.StoreValidationSetsCommand,
            commands.StoreLegacyValidateCommand,
            commands.StoreLegacyGatedCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Store Confdb Schemas",
        [
            commands.StoreEditConfdbSchemaCommand,
            commands.StoreListConfdbSchemasCommand,  # hidden (alias to confdb-schemas)
            commands.StoreConfdbSchemasCommand,
        ],
    ),
    craft_cli.CommandGroup(
        "Other",
        [
            commands.LintCommand,
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
    craft_cli_command_groups = [*COMMAND_GROUPS, CORE22_LIFECYCLE_COMMAND_GROUP]

    return craft_cli.Dispatcher(
        "snapcraft",
        craft_cli_command_groups,
        summary="Package, distribute, and update snaps for Linux and IoT",
        extra_global_args=GLOBAL_ARGS,
        default_command=commands.core22.PackCommand,
    )


def _run_dispatcher(
    dispatcher: craft_cli.Dispatcher, global_args: dict[str, Any]
) -> None:
    if global_args.get("trace"):
        emit.message(
            "Options -t and --trace are deprecated, use --verbosity=debug instead."
        )
        emit.set_mode(EmitterMode.DEBUG)

    # Load the command with a dummy app config to silence deprecation warnings.
    # This config should not actually get used down the line, so its content
    # shouldn't matter
    dispatcher.load_command({"app": "snapcraft_legacy", "services": {}})
    dispatcher.run()
    emit.ended_ok()


def _emit_error(error: craft_cli.CraftError, cause: BaseException | None = None):
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
        print(err, file=sys.stderr)  # to stderr, as argparse normally does
        emit.ended_ok()
        retcode = 1
    except ProvideHelpException as err:
        print(err, file=sys.stderr)  # to stderr, as argparse normally does
        emit.ended_ok()
        retcode = 0
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
                docs_url="https://documentation.ubuntu.com/snapcraft/stable/how-to/publishing/authenticate",
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
                docs_url="https://documentation.ubuntu.com/snapcraft/stable/explanation/remote-build",
            )
        )
        retcode = 1
    except errors.SnapcraftError as err:
        _emit_error(err)
        retcode = 1

    return retcode
