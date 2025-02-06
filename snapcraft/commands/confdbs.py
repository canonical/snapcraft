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

"""Snapcraft Store Confdb Sets commands."""

import argparse
import textwrap

import craft_application.commands
from typing_extensions import override

from snapcraft import const, services


class StoreListConfdbsCommand(craft_application.commands.AppCommand):
    """List confdbs."""

    name = "list-confdbs"
    help_msg = "List confdbs sets"
    overview = textwrap.dedent(
        """
        List all confdbs for the authenticated account.

        Shows the account ID, name, revision, and last modified date of each confdb.

        If a name is provided, only the confdb with that name will be listed.

        Use the ``edit-confdbs`` command create and edit confdbs.
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
            help="Name of the confdb to list",
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
        self._services.confdbs.list_assertions(
            name=parsed_args.name,
            output_format=parsed_args.format,
        )


class StoreEditConfdbsCommand(craft_application.commands.AppCommand):
    """Edit a confdbs set."""

    name = "edit-confdbs"
    help_msg = "Edit or create a confdbs set"
    overview = textwrap.dedent(
        """
        Edit a confdbs set.

        If the confdbs set does not exist, then a new confdbs set will be created.

        If a key name is not provided, the default key is used.

        The account ID of the authenticated account can be determined with the
        ``snapcraft whoami`` command.

        Use the ``list-confdbs`` command to view existing confdbs.
        """
    )
    _services: services.SnapcraftServiceFactory  # type: ignore[reportIncompatibleVariableOverride]

    @override
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "account_id",
            metavar="account-id",
            help="The account ID of the confdbs set to edit",
        )
        parser.add_argument(
            "name", metavar="name", help="Name of the confdbs set to edit"
        )
        parser.add_argument(
            "--key-name", metavar="key-name", help="Key used to sign the confdbs set"
        )

    @override
    def run(self, parsed_args: "argparse.Namespace"):
        self._services.confdbs.edit_assertion(
            name=parsed_args.name,
            account_id=parsed_args.account_id,
            key_name=parsed_args.key_name,
        )
