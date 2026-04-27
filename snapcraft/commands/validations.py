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

import datetime
import re
import textwrap
from typing import TYPE_CHECKING

from craft_application.commands import AppCommand
from craft_cli import emit
from tabulate import tabulate
from typing_extensions import override

from snapcraft import models, services, store

if TYPE_CHECKING:
    import argparse


class StoreGatedCommand(AppCommand):
    """Get snaps and revisions gated for a snap."""

    name = "gated"
    help_msg = "List all gated snaps for <snap-name>"
    overview = textwrap.dedent(
        """
        Get the list of snaps and revisions gated for a snap"""
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
                name = v.approved_snap_name
                revision: str | None = v.approved_snap_revision
                if revision == "-":
                    revision = None
                if v.required is None:
                    required = "True"
                else:
                    required = str(v.required)
                # Currently timestamps have microseconds, which look bad
                timestamp = v.timestamp
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


class StoreValidateCommand(AppCommand):
    """Validate a gated snap."""

    name = "validate"
    help_msg = "Validate a gated snap"
    overview = textwrap.dedent(
        """
        Each validation can be specified with either syntax:

        -  <snap-name>=<revision>
        -  <snap-id>=<revision>"""
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--key-name",
            metavar="key-name",
            help="key used to sign the assertion",
        )
        parser.add_argument(
            "--revoke",
            action="store_true",
            help="revoke validations",
        )
        parser.add_argument(
            "snap_name",
            metavar="snap-name",
        )
        parser.add_argument(
            "validations",
            nargs="+",
        )

    @override
    def run(self, parsed_args: argparse.Namespace):
        """Generate, sign and upload validation assertions."""

        # check validations format
        self._check_validations(parsed_args.validations)

        store_client = store.StoreClientCLI()
        account_info = store_client.get_account_info()
        authority_id = account_info["account_id"]

        # get data for the gating snap
        try:
            snap_id = account_info["snaps"][store.constants.DEFAULT_SERIES][
                parsed_args.snap_name
            ]["snap-id"]
        except KeyError:
            raise store.errors.SnapNotFoundError(snap_name=parsed_args.snap_name)

        existing_validations = {
            (i.approved_snap_id, i.approved_snap_revision): i
            for i in store_client.list_validations(
                snap_id=snap_id,
                params={"include_revoked": "1"},
            )
        }

        # Then, for each requested validation, generate assertion
        for validation in parsed_args.validations:
            gated_snap, rev = validation.split("=", 1)
            emit.progress(f"Getting details for {gated_snap}")
            # The Info API is not authed, so it cannot see private snaps.
            try:
                approved_snap = store_client.get_snap_info(
                    gated_snap, params={"fields": "snap-id"}
                )
                approved_snap_id = approved_snap["snap-id"]
            except store.errors.SnapNotFoundError:
                approved_snap_id = gated_snap

            existing = existing_validations.get((approved_snap_id, rev))
            previous_revision = existing.revision or 0 if existing else 0

            # This uses the same formatting that `datetime.utcnow()` produced before it was deprecated.
            # We've had trouble changing time formats in the past (#5413), so we'll keep
            # using this exact format because we know it's compatible with the store.
            timestamp = datetime.datetime.now(datetime.timezone.utc).strftime(
                "%Y-%m-%dT%H:%M:%S.%fZ"
            )

            # ty and pyright aren't aware of pydantic aliases (https://github.com/astral-sh/ty/issues/1438)
            assertion = models.ValidationAssertion(  # pyright: ignore[reportCallIssue]  # ty: ignore[missing-argument]
                assertion_type="validation",  # pyright: ignore[reportCallIssue]  # ty: ignore[unknown-argument]
                authority_id=authority_id,
                series=store.constants.DEFAULT_SERIES,
                snap_id=snap_id,
                approved_snap_id=approved_snap_id,
                approved_snap_revision=rev,
                timestamp=timestamp,
                revoked=parsed_args.revoke,
                revision=previous_revision + 1 if existing else None,
            )

            signed_validation = services.Assertion.sign_assertion(
                assertion, parsed_args.key_name
            )

            # Save assertion to a properly named file
            fname = f"{parsed_args.snap_name}-{gated_snap}-r{rev}.assertion"
            with open(fname, "wb") as f:
                f.write(signed_validation)

            store_client.post_validation(snap_id, signed_validation)

    def _check_validations(self, validations: list[str]):
        """Check validation strings.

        Matches simple 'key=value' pairs where:
        - key is one or more characters, not containing '='
        - value is a non-negative integer
        """
        validation_re = re.compile("^[^=]+=[0-9]+$")
        invalids = [v for v in validations if not validation_re.match(v)]
        if invalids:
            raise store.errors.InvalidValidationRequestsError(invalids)
