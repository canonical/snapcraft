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

import enum
from typing import Any, Dict, List, Optional, Union

import attr

"""
This module holds representations for request and results for the metrics API
endpoint provided by the Snap Store.

The relevant API documentation is available at:
https://dashboard.snapcraft.io/docs/api/snap.html?#fetch-metrics-for-snaps
"""


MetricsStatus = enum.Enum(  # type: ignore
    value="MetricsStatus",
    names=[("OK", "OK"), ("FAIL", "FAIL"), ("NO DATA", "NO DATA")],
)


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


@attr.s(auto_attribs=True)
class MetricsFilter:
    snap_id: str
    metric_name: str
    start: str
    end: str

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
            isinstance(b, str) or isinstance(b, int) or b is None for b in values
        ):
            raise ValueError(f"Invalid metric values: {values!r}")

        currently_released = payload.get("currently_released")
        if currently_released is not None and not isinstance(currently_released, bool):
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

    def marshal(self) -> Dict[str, Any]:
        return {
            "status": self.status.name,
            "snap_id": self.snap_id,
            "metric_name": self.metric_name,
            "buckets": self.buckets,
            "series": [s.marshal() for s in self.series],
        }

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "MetricResults":
        status_string = payload.get("status", "")
        try:
            status = MetricsStatus[status_string]
        except (KeyError, TypeError):
            raise ValueError(f"Invalid metric status: {status_string!r}")

        snap_id = payload.get("snap_id")
        if not isinstance(snap_id, str):
            raise ValueError(f"Invalid metric snap id: {snap_id!r}")

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

    def marshal(self) -> Dict[str, Any]:
        return {"metrics": [m.marshal() for m in self.metrics]}

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "MetricsResults":
        if not isinstance(payload, dict):
            raise ValueError(f"Invalid metrics results: {payload!r}")

        metrics = payload.get("metrics")
        if not isinstance(metrics, list):
            raise ValueError(f"Invalid metrics: {metrics!r}")

        return cls(metrics=[MetricResults.unmarshal(m) for m in metrics])
