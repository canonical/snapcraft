# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Snapcraft Store Confdb Schemas commands."""

from __future__ import annotations

import textwrap
from typing import TYPE_CHECKING

import craft_application.commands
from typing_extensions import override

from snapcraft import const, errors, services

if TYPE_CHECKING:
    import argparse


class StoreConfdbSchemasCommand(craft_application.commands.AppCommand):
    """List confdb schemas."""

    name = "confdb-schemas"
    help_msg = "List confdb schemas"
    overview = textwrap.dedent(
        """
        List all confdb schemas for the authenticated account.

        Shows the account ID, name, revision, and last modified date of each confdb-schema.

        If a name is provided, only the confdb schema with that name will be listed.

        Use the ``edit-confdb-schema`` command create and edit confdb schemas.
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
            help="Name of the confdb schema to list",
        )
        parser.add_argument(
            "--format",
            type=str,
            choices=[
                const.OutputFormat.table.value,
                const.OutputFormat.json.value,
            ],
            help="The output format (table | json)",
            default=const.OutputFormat.table,
        )

    @override
    def run(self, parsed_args: argparse.Namespace):
        self._services.confdb_schemas.list_assertions(
            name=parsed_args.name,
            output_format=parsed_args.format,
        )


class StoreListConfdbSchemasCommand(StoreConfdbSchemasCommand):
    """Removed command alias to list confdb schemas."""

    name = "list-confdb-schemas"
    hidden = True

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)


class StoreEditConfdbSchemaCommand(craft_application.commands.AppCommand):
    """Edit a confdb schema."""

    name = "edit-confdb-schema"
    help_msg = "Edit or create a confdb-schema"
    overview = textwrap.dedent(
        """
        Edit a confdb schema.

        If the confdb schema does not exist, then a new confdb schema will be created.

        If a key name is not provided, the default key is used.

        The account ID of the authenticated account can be determined with the
        ``snapcraft whoami`` command.

        Use the ``confdb-schemas`` command to view existing confdb schemas.
        """
    )
    _services: services.SnapcraftServiceFactory  # type: ignore[reportIncompatibleVariableOverride]

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "account_id",
            metavar="account-id",
            help="The account ID of the confdb schema to edit",
        )
        parser.add_argument(
            "name", metavar="name", help="Name of the confdb schema to edit"
        )
        parser.add_argument(
            "--key-name", metavar="key-name", help="Key used to sign the confdb schema"
        )

    @override
    def run(self, parsed_args: argparse.Namespace):
        self._services.confdb_schemas.edit_assertion(
            name=parsed_args.name,
            account_id=parsed_args.account_id,
            key_name=parsed_args.key_name,
        )
