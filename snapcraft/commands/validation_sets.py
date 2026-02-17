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

if TYPE_CHECKING:
    import argparse

    from snapcraft import services


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
