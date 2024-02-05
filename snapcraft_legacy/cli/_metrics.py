# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd.
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

import logging
from typing import List, Union

from packaging import version

from snapcraft_legacy.storeapi import metrics as metrics_module

logger = logging.getLogger(__name__)


METRIC_NAMES_TO_SERIES_LABEL_MAPPINGS = {
    metrics_module.MetricsNames.DAILY_DEVICE_CHANGE.value: "Devices",
    metrics_module.MetricsNames.INSTALLED_BASE_BY_CHANNEL.value: "Channel",
    metrics_module.MetricsNames.INSTALLED_BASE_BY_COUNTRY.value: "Country",
    metrics_module.MetricsNames.INSTALLED_BASE_BY_OPERATING_SYSTEM.value: "OS",
    metrics_module.MetricsNames.INSTALLED_BASE_BY_VERSION.value: "Version",
    metrics_module.MetricsNames.WEEKLY_DEVICE_CHANGE.value: "Devices",
    metrics_module.MetricsNames.WEEKLY_INSTALLED_BASE_BY_CHANNEL.value: "Channel",
    metrics_module.MetricsNames.WEEKLY_INSTALLED_BASE_BY_COUNTRY.value: "Country",
    metrics_module.MetricsNames.WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM.value: "OS",
    metrics_module.MetricsNames.WEEKLY_INSTALLED_BASE_BY_VERSION.value: "Version",
}


def get_series_label_from_metric_name(metric_name: str) -> str:
    """Get series label from metric name.

    :returns: Label for series.

    :raises KeyError: if metric name not supported.
    """
    return METRIC_NAMES_TO_SERIES_LABEL_MAPPINGS[metric_name]


def convert_metrics_to_table(
    results: metrics_module.MetricResults, *, transpose: bool = True
) -> List[List[Union[str, int]]]:
    rows: List[List[Union[str, int]]] = []

    if results.status == metrics_module.MetricsStatus["NO DATA"]:
        # No data available, return empty list.
        return rows
    elif results.status == metrics_module.MetricsStatus["FAIL"]:
        # API docs say to consider this as data to discard, just warn the user.
        logger.warning("No data available due to Snap Store internal failure.")
        return rows

    # Sort series sensibly, using a version sort.
    def key_as_version(series):
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
        initial_column_header = get_series_label_from_metric_name(results.metric_name)

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
