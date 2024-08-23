# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2024 Canonical Ltd.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License version 3, as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
#  SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""Registry set commands."""

import argparse
import textwrap

from craft_application.commands import AppCommand
from typing_extensions import override


class StoreEditRegistrySetsCommand(AppCommand):
    """Edit a validation set."""

    name = "edit-registry-sets"
    help_msg = "Edit registry sets"
    overview = textwrap.dedent(
        """
        For more information on registry sets, see
        https://canonical-snapcraft.readthedocs-hosted.com/en/latest/reference/registry_sets.html
        """
    )

    @override
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--name",
            metavar="name",
            type=str,
            default=None,
            nargs="?",
            help="Name of registry set to edit",
        )

    @override
    def run(self, parsed_args: "argparse.Namespace"):
        self._services.RegistrySetsService.edit_registry_sets(parsed_args.name)


class StoreListRegistrySetsCommand(AppCommand):
    """List registry sets."""

    name = "list-registry-sets"
    help_msg = "List all registry sets"
    overview = textwrap.dedent(
        """
        For more information on registry sets, see
        https://canonical-snapcraft.readthedocs-hosted.com/en/latest/reference/registry_sets.html
        """
    )

    @override
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        parser.add_argument(
            "--name",
            metavar="name",
            required=False,
            type=str,
            help="Name of registry set to list",
        )

    @override
    def run(self, parsed_args: "argparse.Namespace"):
        self._services.RegistrySetsService.list_registry_sets(parsed_args.name)
