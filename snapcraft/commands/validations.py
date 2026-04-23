# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Snapcraft Store Validation commands."""

from __future__ import annotations

import textwrap
from typing import TYPE_CHECKING

from craft_application.commands import AppCommand
from craft_cli import emit
from tabulate import tabulate
from typing_extensions import override

from snapcraft import store

if TYPE_CHECKING:
    import argparse


class StoreGatedCommand(AppCommand):
    """Get snaps and revisions gating a snap."""

    name = "gated"
    help_msg = "List all gated snaps for <snap-name>"
    overview = textwrap.dedent(
        """
        Get the list of snaps and revisions gating a snap"""
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument("snap_name", metavar="snap-name")

    @override
    def run(self, parsed_args: argparse.Namespace):
        """Print list of snaps gated by snap_name."""
        store_client = store.StoreClientCLI()
        account_info = store_client.get_account_info()
        # Get data for the gating snap
        snaps = account_info.get("snaps", {})

        # Resolve name to snap-id
        try:
            snap_id = snaps[store.constants.DEFAULT_SERIES][parsed_args.snap_name][
                "snap-id"
            ]
        except KeyError:
            raise store.errors.SnapNotFoundError(snap_name=parsed_args.snap_name)

        validations = store_client.list_validations(snap_id)

        if validations:
            table_data = []
            for v in validations:
                name = v["approved-snap-name"]
                revision = v["approved-snap-revision"]
                if revision == "-":
                    revision = None
                required = str(v.get("required", True))
                # Currently timestamps have microseconds, which look bad
                timestamp = v["timestamp"]
                if "." in timestamp:
                    timestamp = timestamp.split(".")[0] + "Z"
                table_data.append([name, revision, required, timestamp])
            tabulated = tabulate(
                table_data,
                headers=["Name", "Revision", "Required", "Approved"],
                tablefmt="plain",
                missingval="-",
            )
            emit.message(tabulated)
        else:
            emit.message(f"There are no validations for snap {parsed_args.snap_name!r}")
