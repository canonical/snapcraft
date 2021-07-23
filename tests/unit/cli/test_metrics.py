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

import pytest

from snapcraft.cli._metrics import (
    convert_metrics_to_table,
    get_series_label_from_metric_name,
)
from snapcraft.storeapi import metrics


def test_get_series_label_from_metric_name():
    mappings = {
        n.value: get_series_label_from_metric_name(n.value)
        for n in metrics.MetricsNames
    }

    assert mappings == {
        "daily_device_change": "Devices",
        "installed_base_by_channel": "Channel",
        "installed_base_by_country": "Country",
        "installed_base_by_operating_system": "OS",
        "installed_base_by_version": "Version",
        "weekly_device_change": "Devices",
        "weekly_installed_base_by_channel": "Channel",
        "weekly_installed_base_by_country": "Country",
        "weekly_installed_base_by_operating_system": "OS",
        "weekly_installed_base_by_version": "Version",
    }


@pytest.mark.parametrize("metric_name", [n.value for n in metrics.MetricsNames])
def test_convert_metrics_to_table_no_data(caplog, metric_name):
    results = metrics.MetricResults(
        status=metrics.MetricsStatus["NO DATA"],
        snap_id="foo",
        metric_name=metric_name,
        buckets=[],
        series=[],
    )

    assert convert_metrics_to_table(results) == []
    assert [rec.message for rec in caplog.records] == []


@pytest.mark.parametrize("metric_name", [n.value for n in metrics.MetricsNames])
def test_convert_metrics_to_table_store_failure_no_data(caplog, metric_name):
    results = metrics.MetricResults(
        status=metrics.MetricsStatus["FAIL"],
        snap_id="foo",
        metric_name=metric_name,
        buckets=[],
        series=[],
    )

    assert convert_metrics_to_table(results) == []
    assert [rec.message for rec in caplog.records] == [
        "No data available due to Snap Store internal failure."
    ]


@pytest.mark.parametrize("metric_name", [n.value for n in metrics.MetricsNames])
def test_convert_metrics_to_table_store_one_bucket(caplog, metric_name):
    results = metrics.MetricResults(
        status=metrics.MetricsStatus["OK"],
        snap_id="test-snap-id",
        metric_name=metric_name,
        buckets=["2021-01-01"],
        series=[metrics.Series(name="blah", values=[1], currently_released=None)],
    )

    assert convert_metrics_to_table(results) == [
        (get_series_label_from_metric_name(metric_name), "2021-01-01"),
        ("Blah", 1),
    ]
    assert [rec.message for rec in caplog.records] == []


@pytest.mark.parametrize("metric_name", [n.value for n in metrics.MetricsNames])
def test_convert_metrics_to_table_store_multiple_buckets(caplog, metric_name):
    results = metrics.MetricResults(
        status=metrics.MetricsStatus["OK"],
        snap_id="test-snap-id",
        metric_name=metric_name,
        buckets=["2021-01-01", "2021-01-02", "2021-01-03"],
        series=[
            metrics.Series(name="blah", values=[1, 2, 3], currently_released=None),
            metrics.Series(name="blahh", values=[4, 5, 6], currently_released=None),
            metrics.Series(name="blahhh", values=[7, 8, 9], currently_released=None),
        ],
    )

    assert convert_metrics_to_table(results) == [
        (
            get_series_label_from_metric_name(metric_name),
            "2021-01-01",
            "2021-01-02",
            "2021-01-03",
        ),
        ("Blah", 1, 2, 3),
        ("Blahh", 4, 5, 6),
        ("Blahhh", 7, 8, 9),
    ]
    assert [rec.message for rec in caplog.records] == []
