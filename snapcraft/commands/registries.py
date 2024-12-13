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

"""Snapcraft Store Registry Sets commands."""

import argparse
import textwrap

import craft_application.commands
from typing_extensions import override

from snapcraft import const, services


class StoreListRegistriesCommand(craft_application.commands.AppCommand):
    """List registries."""

    name = "list-registries"
    help_msg = "List registries sets"
    overview = textwrap.dedent(
        """
        List all registries for the authenticated account.

        Shows the account ID, name, revision, and last modified date of each registry.

        If a name is provided, only the registry with that name will be listed.

        Use the ``edit-registries`` command create and edit registries.
        """
    )
    _services: services.SnapcraftServiceFactory  # type: ignore[reportIncompatibleVariableOverride]

    @override
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--name",
            metavar="name",
            required=False,
            type=str,
            help="Name of the registry to list",
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
    def run(self, parsed_args: "argparse.Namespace"):
        self._services.registries.list_assertions(
            name=parsed_args.name,
            output_format=parsed_args.format,
        )


class StoreEditRegistriesCommand(craft_application.commands.AppCommand):
    """Edit a registries set."""

    name = "edit-registries"
    help_msg = "Edit or create a registries set"
    overview = textwrap.dedent(
        """
        Edit a registries set.

        If the registries set does not exist, then a new registries set will be created.

        If a key name is not provided, the default key is used.

        The account ID of the authenticated account can be determined with the
        ``snapcraft whoami`` command.

        Use the ``list-registries`` command to view existing registries.
        """
    )
    _services: services.SnapcraftServiceFactory  # type: ignore[reportIncompatibleVariableOverride]

    @override
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "account_id",
            metavar="account-id",
            help="The account ID of the registries set to edit",
        )
        parser.add_argument(
            "name", metavar="name", help="Name of the registries set to edit"
        )
        parser.add_argument(
            "--key-name", metavar="key-name", help="Key used to sign the registries set"
        )

    @override
    def run(self, parsed_args: "argparse.Namespace"):
        self._services.registries.edit_assertion(
            name=parsed_args.name,
            account_id=parsed_args.account_id,
            key_name=parsed_args.key_name,
        )
