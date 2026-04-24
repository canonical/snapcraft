# Copyright 2026 Canonical Ltd
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

"""Models for snap metrics."""

from enum import Enum
from typing import Literal

from craft_application import models
from pydantic import Field


class MetricName(str, Enum):
    DAILY_DEVICE_CHANGE = "daily_device_change"
    INSTALLED_BASE_BY_ARCHITECTURE = "installed_base_by_architecture"
    INSTALLED_BASE_BY_CHANNEL = "installed_base_by_channel"
    INSTALLED_BASE_BY_COUNTRY = "installed_base_by_country"
    INSTALLED_BASE_BY_OPERATING_SYSTEM = "installed_base_by_operating_system"
    INSTALLED_BASE_BY_VERSION = "installed_base_by_version"
    WEEKLY_DEVICE_CHANGE = "weekly_device_change"
    WEEKLY_INSTALLED_BASE_BY_ARCHITECTURE = "weekly_installed_base_by_architecture"
    WEEKLY_INSTALLED_BASE_BY_CHANNEL = "weekly_installed_base_by_channel"
    WEEKLY_INSTALLED_BASE_BY_COUNTRY = "weekly_installed_base_by_country"
    WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM = (
        "weekly_installed_base_by_operating_system"
    )
    WEEKLY_INSTALLED_BASE_BY_VERSION = "weekly_installed_base_by_version"

    def __str__(self) -> str:
        return self.value

    def to_label(self) -> str:
        match self:
            case self.DAILY_DEVICE_CHANGE | self.WEEKLY_DEVICE_CHANGE:
                return "Devices"
            case self.INSTALLED_BASE_BY_CHANNEL | self.WEEKLY_INSTALLED_BASE_BY_CHANNEL:
                return "Channel"
            case self.INSTALLED_BASE_BY_COUNTRY | self.WEEKLY_INSTALLED_BASE_BY_COUNTRY:
                return "Country"
            case (
                self.INSTALLED_BASE_BY_OPERATING_SYSTEM
                | self.WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM
            ):
                return "OS"
            case self.INSTALLED_BASE_BY_VERSION | self.WEEKLY_INSTALLED_BASE_BY_VERSION:
                return "Version"
            case (
                self.INSTALLED_BASE_BY_ARCHITECTURE
                | self.WEEKLY_INSTALLED_BASE_BY_ARCHITECTURE
            ):
                return "Architecture"
            case _:
                raise NotImplementedError


class Series(models.CraftBaseModel):
    """Metrics timeseries."""

    name: str
    """Category of data being represented for a given metric."""

    values: list[int | str]
    """Data points for this series."""

    currently_released: bool | None = None
    """Optional value stating whether a given channel is currently released when
    representing a "_by_channel" metric series."""


class Metric(models.CraftBaseModel):
    """Single point of metrics for a snap."""

    status: Literal["OK", "FAIL", "NO_DATA"]
    """Status of metrics retrieval."""

    snap_id: str = Field(serialization_alias="snap_id")
    """ID of checked snap."""

    metric_name: MetricName = Field(serialization_alias="metric_name")
    """Type of metric data being represented."""

    buckets: list[str]
    """List of dates. buckets[x] corresponds to any given series[y].values[x]"""

    series: list[Series]
    """List of available timeseries in the context of the requested metric."""


class MetricsResponse(models.CraftBaseModel):
    """Store response from the metrics API endpoint."""

    metrics: list[Metric]
    """Metrics returned by the Snap Store."""
