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

"""Command for retrieving snap metrics."""

from __future__ import annotations

import json
import textwrap
from datetime import date, timedelta
from typing import TYPE_CHECKING

from craft_application.commands import AppCommand
from craft_cli import emit
from packaging import version
from tabulate import tabulate
from typing_extensions import override

from snapcraft import store
from snapcraft.models import Metric, MetricName, Series

if TYPE_CHECKING:
    import argparse

_YESTERDAY = str(date.today() - timedelta(days=1))


class StoreMetricsCommand(AppCommand):
    """Get metrics for a given snap."""

    name = "metrics"
    help_msg = "Retrieve store metrics for a snap"
    overview = textwrap.dedent(
        """
        Retrieve store metric data for a snap you own. New metrics are generated
        daily."""
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "snap_name",
            metavar="snap-name",
            type=str,
            help="The snap name to check",
        )
        parser.add_argument(
            "--name",
            metavar="metric",
            dest="metric",
            help="Metric to view",
            type=str,
            choices=[metric.value for metric in MetricName],
            required=True,
        )
        parser.add_argument(
            "--start",
            metavar="date",
            help="Date in the format of YYYY-MM-DD. Defaults to yesterday.",
            default=_YESTERDAY,
        )
        parser.add_argument(
            "--end",
            metavar="date",
            help="Date in the format of YYYY-MM-DD. Defaults to yesterday.",
            default=_YESTERDAY,
        )
        parser.add_argument(
            "--format",
            metavar="format",
            help="Format for output",
            type=str,
            choices=["table", "json"],
            required=True,
        )

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        store_client = store.StoreClientCLI()
        account_info = store_client.get_account_info()

        try:
            snap_id = account_info["snaps"][store.constants.DEFAULT_SERIES][
                parsed_args.snap_name
            ]["snap-id"]
        except KeyError:
            raise store.errors.SnapNotFoundError(snap_name=parsed_args.snap_name)

        results = store_client.get_metrics(
            filters=[
                {
                    "snap_id": snap_id,
                    "metric_name": parsed_args.metric,
                    "start": parsed_args.start,
                    "end": parsed_args.end,
                }
            ],
        )

        metrics = results.metrics

        if len(metrics) != 1:
            raise RuntimeError(f"Unexpected metric results from store: {results!r}")

        metric_results = metrics[0]

        if parsed_args.format == "json":
            output = json.dumps(metric_results.marshal(), indent=2, sort_keys=True)
            emit.message(output)
        elif parsed_args.format == "table":
            rows = self.convert_metrics_to_table(metric_results, transpose=True)
            output = tabulate(rows, tablefmt="plain")
            emit.message(output)

    @staticmethod
    def convert_metrics_to_table(
        results: Metric, *, transpose: bool = True
    ) -> list[list[str | int]]:
        rows: list[list[str | int]] = []

        status = results.status
        if status == "NO_DATA":
            # No data available, return empty list.
            return rows
        if status == "FAIL":
            # API docs say to consider this as data to discard, just warn the user.
            emit.warning("No data available due to Snap Store internal failure.")
            return rows

        # Sort series sensibly, using a version sort.
        def key_as_version(series: Series):
            """Key series as versions, if possible."""
            try:
                return version.parse(series.name)
            except version.InvalidVersion:
                return version.Version("0.0")

        series = sorted(results.series, key=key_as_version)

        # Determine headers from series, taking into account whether table will be transposed.
        if not transpose:
            initial_column_header = "Date"
        else:
            initial_column_header = results.metric_name.to_label()

        header_row = [
            initial_column_header,
            *[s.name.capitalize() for s in series],
        ]

        # First add header row.
        rows.append(header_row)  # type: ignore

        # Add data rows.
        for i, bucket in enumerate(results.buckets):
            rows.append([bucket] + [s.values[i] or 0 for s in series])  # type: ignore

        # Optionally transpose.
        if transpose:
            rows = list(zip(*rows))  # type: ignore

        return rows
