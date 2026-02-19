# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Snapcraft Store Validation Sets commands."""

from __future__ import annotations

import textwrap
from typing import TYPE_CHECKING

from craft_application.commands import AppCommand
from typing_extensions import override

from snapcraft import const, errors

if TYPE_CHECKING:
    import argparse

    from snapcraft import services


class StoreValidationSetsCommand(AppCommand):
    """List validation sets."""

    name = "validation-sets"
    help_msg = "List validation sets"
    overview = textwrap.dedent(
        """
        List all validation sets for the authenticated account.

        Shows the account ID, name, sequence, revision, and last modified date of each validation set.

        If a name is provided, only the validation set with that name is listed.

        Use the 'edit-validation-sets' command to modify validation sets.
        """
    )
    _services: services.SnapcraftServiceFactory  # type: ignore[reportIncompatibleVariableOverride]

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--name",
            metavar="name",
            required=False,
            type=str,
            help="Name of the validation set to list",
        )
        parser.add_argument(
            "--sequence",
            metavar="sequence",
            # the store will return the latest sequences by default
            required=False,
            type=str,
            choices=["all", "latest"],
            help="Sequences to list (default is 'latest', options are 'all', 'latest').",
        )
        parser.add_argument(
            "--format",
            type=str,
            choices=const.OUTPUT_FORMATS,
            help="The output format (default is 'table', options are 'table', 'json').",
            default=const.OutputFormat.table,
        )

    @override
    def run(self, parsed_args: argparse.Namespace):
        kwargs = {"sequence": parsed_args.sequence} if parsed_args.sequence else {}
        self._services.validation_sets.list_assertions(
            name=parsed_args.name,
            output_format=parsed_args.format,
            **kwargs,
        )


class StoreListValidationSetsCommand(StoreValidationSetsCommand):
    """Removed command alias for the validation-sets command."""

    name = "list-validation-sets"
    hidden = True

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)


class StoreEditValidationSetsCommand(AppCommand):
    """Edit a validation set."""

    name = "edit-validation-sets"
    help_msg = "Edit a validation set."
    overview = textwrap.dedent(
        """
        Edit a validation set.

        If the validation set does not exist, then a new one is created.

        If a key name is not provided, the default key is used.

        The account ID of the authenticated account can be determined with the
        'snapcraft whoami' command.

        Use the 'validation-sets' command to view existing validation sets.
        """
    )
    _services: services.SnapcraftServiceFactory  # type: ignore[reportIncompatibleVariableOverride]

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--key-name", metavar="key-name", help="Key used to sign the assertion"
        )
        parser.add_argument("account_id", metavar="account-id")
        parser.add_argument("name", metavar="name")
        parser.add_argument("sequence", metavar="sequence", type=int)

    @override
    def run(self, parsed_args: argparse.Namespace):
        self._services.validation_sets.edit_assertion(
            name=parsed_args.name,
            account_id=parsed_args.account_id,
            key_name=parsed_args.key_name,
            sequence=parsed_args.sequence,
        )
