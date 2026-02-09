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

"""Snapcraft Store Account management commands."""

from __future__ import annotations

import json
import operator
import textwrap
from typing import TYPE_CHECKING

from craft_application.commands import AppCommand
from craft_cli import emit
from overrides import overrides
from tabulate import tabulate

from snapcraft import errors, store, utils

if TYPE_CHECKING:
    import argparse


_MESSAGE_REGISTER_PRIVATE = textwrap.dedent(
    """\
    Even though this is private snap, you should think carefully about
    the choice of name and make sure you are confident nobody else will
    have a stronger claim to that particular name. If you are unsure
    then we suggest you prefix the name with your developer identity,
    As '$username-yoyodyne-www-site-content'."""
)
_MESSAGE_REGISTER_CONFIRM = textwrap.dedent(
    """\
    We always want to ensure that users get the software they expect
    for a particular name.

    If needed, we will rename snaps to ensure that a particular name
    reflects the software most widely expected by our community.

    For example, most people would expect 'thunderbird' to be published by
    Mozilla. They would also expect to be able to get other snaps of
    Thunderbird as '$username-thunderbird'.

    Would you say that MOST users will expect {!r} to come from
    you, and be the software you intend to publish there?"""
)
_MESSAGE_REGISTER_SUCCESS = "Registered {!r}"
_MESSAGE_REGISTER_NO = "Snap name {!r} not registered"


def _set_nil(value: str):
    """Return None when given the input "-", else the original input."""
    return None if value == "-" else value


class StoreRegisterCommand(AppCommand):
    """Register a snap name with the Snap Store."""

    name = "register"
    help_msg = "Register <snap-name> with the store"
    overview = textwrap.dedent(
        """
        Register an available <snap-name> with the Snap Store,
        at which time you become the publisher for the snap."""
    )

    @overrides
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "snap-name",
            type=str,
            help="The snap name to register",
        )
        parser.add_argument(
            "--store",
            metavar="<store-id>",
            dest="store_id",
            type=str,
            default=None,
            help="Store to register with",
        )
        parser.add_argument(
            "--private",
            action="store_true",
            default=False,
            help="Register the snap as a private one",
        )
        parser.add_argument(
            "--yes",
            action="store_true",
            default=False,
            help="Do not ask for confirmation",
        )

    @overrides
    def run(self, parsed_args: argparse.Namespace):
        # dest does not work when filling the parser so getattr instead
        snap_name = getattr(parsed_args, "snap-name")

        if parsed_args.private:
            emit.progress(
                _MESSAGE_REGISTER_PRIVATE.format(snap_name),
                permanent=True,
            )
        if parsed_args.yes or utils.confirm_with_user(
            _MESSAGE_REGISTER_CONFIRM.format(snap_name)
        ):
            store.StoreClientCLI().register(
                snap_name, is_private=parsed_args.private, store_id=parsed_args.store_id
            )
            emit.message(_MESSAGE_REGISTER_SUCCESS.format(snap_name))
        else:
            emit.message(_MESSAGE_REGISTER_NO.format(snap_name))


class StoreNamesCommand(AppCommand):
    """List the snap names registered with the current account."""

    name = "names"
    help_msg = "List the names registered to the logged-in account"
    overview = textwrap.dedent(
        """
        Return the list of snap names together with the registration date,
        visibility and any additional notes."""
    )

    @overrides
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--format",
            type=str,
            choices=["json", "table"],
            help="The output format (table | json)",
            default="table",
        )

    @overrides
    def run(self, parsed_args: argparse.Namespace):
        store_client = store.StoreClientCLI()
        snaps = store_client.get_names()
        snaps.sort(key=operator.itemgetter(0))

        if not snaps:
            emit.message("No registered snaps")
        else:
            headers = ["Name", "Since", "Visibility", "Notes"]

            if parsed_args.format == "table":
                tabulated_snaps = tabulate(
                    snaps,
                    headers=headers,
                    tablefmt="plain",
                )
                emit.message(tabulated_snaps)
            elif parsed_args.format == "json":
                json_snaps = {
                    "snaps": [
                        {
                            header.lower(): _set_nil(value)
                            for header, value in zip(headers, snap)
                        }
                        for snap in snaps
                    ]
                }
                emit.message(json.dumps(json_snaps, indent=4))
            else:
                raise NotImplementedError("Format not implemented", parsed_args.format)


class StoreLegacyListCommand(StoreNamesCommand):
    """Removed command to list the snap names registered with the current account."""

    name = "list"
    hidden = True

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)


class StoreLegacyListRegisteredCommand(StoreNamesCommand):
    """Removed command to list the snap names registered with the current account."""

    name = "list-registered"
    hidden = True

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)
