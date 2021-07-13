# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import enum
from typing import Any, Dict, List, Optional, Union

import attr
import pkg_resources

"""
This module holds representations for results for the v2 whoami
API endpoint provided by the Snap Store.

The full API is documented on
https://dashboard.snapcraft.io/docs/v2/en/tokens.html#api-tokens-whoami
"""


class MetricsStatus(enum.Enum):
    OK = "OK"
    FAIL = "FAIL"
    NO_DATA = "NO_DATA"


class MetricsNames(enum.Enum):
    DAILY_DEVICE_CHANGE = "daily_device_change"
    INSTALLED_BASE_BY_CHANNEL = "installed_base_by_channel"
    INSTALLED_BASE_BY_COUNTRY = "installed_base_by_country"
    INSTALLED_BASE_BY_OPERATING_SYSTEM = "installed_base_by_operating_system"
    INSTALLED_BASE_BY_VERSION = "installed_base_by_version"
    WEEKLY_DEVICE_CHANGE = "weekly_device_change"
    WEEKLY_INSTALLED_BASE_BY_CHANNEL = "weekly_installed_base_by_channel"
    WEEKLY_INSTALLED_BASE_BY_COUNTRY = "weekly_installed_base_by_country"
    WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM = (
        "weekly_installed_base_by_operating_system"
    )
    WEEKLY_INSTALLED_BASE_BY_VERSION = "weekly_installed_base_by_version"


_METRIC_NAMES_TO_SERIES_LABEL_MAPPINGS = {
    MetricsNames.DAILY_DEVICE_CHANGE.value: "Devices",
    MetricsNames.INSTALLED_BASE_BY_CHANNEL.value: "Channel",
    MetricsNames.INSTALLED_BASE_BY_COUNTRY.value: "Country",
    MetricsNames.INSTALLED_BASE_BY_OPERATING_SYSTEM.value: "OS",
    MetricsNames.INSTALLED_BASE_BY_VERSION.value: "Version",
    MetricsNames.WEEKLY_DEVICE_CHANGE.value: "Devices",
    MetricsNames.WEEKLY_INSTALLED_BASE_BY_CHANNEL.value: "Channel",
    MetricsNames.WEEKLY_INSTALLED_BASE_BY_COUNTRY.value: "Country",
    MetricsNames.WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM.value: "OS",
    MetricsNames.WEEKLY_INSTALLED_BASE_BY_VERSION.value: "Version",
}


def get_series_label_from_metric_name(metric_name: str) -> str:
    try:
        return _METRIC_NAMES_TO_SERIES_LABEL_MAPPINGS[metric_name]
    except KeyError:
        raise RuntimeError(f"Unhandled metric name: {metric_name}")


@attr.s(auto_attribs=True)
class MetricsFilter:
    snap_id: str
    metric_name: str = "installed_base_by_channel"
    start: str = "2021-01-01"
    end: str = "2021-01-31"

    def marshal(self) -> Dict[str, str]:
        return attr.asdict(self)


@attr.s(auto_attribs=True)
class Series:
    name: str
    values: List[Union[str, int]]
    currently_released: Optional[bool] = None

    def marshal(self) -> Dict[str, str]:
        obj = attr.asdict(self)

        if self.currently_released is None:
            obj.pop("currently_released")

        return obj

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Series":
        name = payload.get("name")
        if not isinstance(name, str):
            raise ValueError(f"Invalid metric name: {name!r}")

        values = payload.get("values")

        if not isinstance(values, list) or not all(
            [isinstance(b, str) or isinstance(b, int) or b is None for b in values]
        ):
            raise ValueError(f"Invalid metric values: {values!r}")

        currently_released = payload.get("currently_released")
        if currently_released is not None and not isinstance(name, str):
            raise ValueError(
                f"Invalid metric currently_released: {currently_released!r}"
            )

        return cls(name=name, values=values, currently_released=currently_released,)


@attr.s(auto_attribs=True)
class MetricResults:
    status: MetricsStatus
    snap_id: str
    metric_name: str
    buckets: List[str]
    series: List[Series]

    def convert_to_table(
        self, *, transpose: bool = False
    ) -> List[List[Union[str, int]]]:
        rows: List[List[Union[str, int]]] = []

        if self.status == MetricsStatus.NO_DATA:
            # No data available, return empty list.
            return rows
        elif self.status == MetricsStatus.FAIL:
            raise RuntimeError("Failed to query requested metric.")

        # Sort series sensibly, using a version sort.
        def key_as_version(series):
            """Key series as versions, if possible."""
            return pkg_resources.parse_version(series.name)

        series = sorted(self.series, key=key_as_version)

        # Determine headers from series, taking into account whether table will be transposed.
        if not transpose:
            initial_column_header = "Date"
        else:
            initial_column_header = get_series_label_from_metric_name(self.metric_name)

        header_row = [
            initial_column_header,
            *[s.name.capitalize() for s in series],
        ]

        # First add header row.
        rows.append(header_row)  # type: ignore

        # Add data rows.
        for i, bucket in enumerate(self.buckets):
            rows.append([bucket] + [s.values[i] or "-" for s in series])  # type: ignore

        # Optionally transpose.
        if transpose:
            rows = list(zip(*rows))  # type: ignore

        return rows

    def marshal(self) -> Dict[str, str]:
        return attr.asdict(self)

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "MetricResults":
        status = payload.get("status")
        if status not in ["OK", "FAIL", "NO_DATA"]:
            raise ValueError(f"Invalid metric status: {status!r}")

        snap_id = payload.get("snap_id")
        if not isinstance(snap_id, str):
            raise ValueError(f"Invalid metric snap_id: {snap_id!r}")

        metric_name = payload.get("metric_name")
        if not isinstance(metric_name, str):
            raise ValueError(f"Invalid metric name: {metric_name!r}")

        buckets = payload.get("buckets")
        if not isinstance(buckets, list) or any(
            [not isinstance(b, str) for b in buckets]
        ):
            raise ValueError(f"Invalid metric buckets: {buckets!r}")

        series = payload.get("series")
        if not isinstance(series, list):
            raise ValueError(f"Invalid metric series: {series!r}")

        return cls(
            status=status,
            snap_id=snap_id,
            metric_name=metric_name,
            buckets=buckets,
            series=[Series.unmarshal(s) for s in series],
        )


@attr.s(auto_attribs=True)
class MetricsResults:
    metrics: List[MetricResults]

    def marshal(self) -> Dict[str, str]:
        return attr.asdict(self)

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "MetricsResults":
        metrics = payload.get("metrics")
        if not isinstance(metrics, list):
            raise ValueError(f"Invalid metrics results: {metrics!r}")

        return cls(metrics=[MetricResults.unmarshal(m) for m in metrics])
