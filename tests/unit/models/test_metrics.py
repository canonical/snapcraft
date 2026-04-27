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
from pydantic import ValidationError

from snapcraft.models import (
    Metric,
    MetricName,
    MetricsResponse,
    Series,
)


@pytest.mark.parametrize("values", [[""], ["x"], [1], ["x", 1]])
@pytest.mark.parametrize("currently_released", [False, True, None])
def test_series(values, currently_released):
    expected_series = Series(
        name="test-name", values=values, currently_released=currently_released
    )

    data = {
        "name": "test-name",
        "values": values,
        "currently_released": currently_released,
    }

    if currently_released is None:
        data.pop("currently_released")

    assert Series.unmarshal(data) == expected_series


@pytest.mark.parametrize("values", [False, 1, "x", {}, [{}]])
def test_series_invalid_values(values):
    series = {"name": "test-name", "values": values}

    with pytest.raises(ValidationError):
        Series.unmarshal(series)


@pytest.mark.parametrize("currently_released", [5, "x", {}, [], [{}]])
def test_series_invalid_currently_released(currently_released):
    series = {
        "name": "test-name",
        "values": [],
        "currently_released": currently_released,
    }

    with pytest.raises(ValidationError):
        Series.unmarshal(series)


@pytest.mark.parametrize("buckets", [[], ["2021-01-01"], ["2021-01-01", "2021-01-02"]])
@pytest.mark.parametrize(
    "series", [[], [{"name": "s1", "values": []}], [{"name": "s2", "values": ["v1"]}]]
)
@pytest.mark.parametrize("status", ["OK", "FAIL", "NO_DATA"])
def test_metric_results(buckets, series, status):
    results = Metric(
        status=status,
        snap_id="test-id",
        metric_name=MetricName.DAILY_DEVICE_CHANGE,
        buckets=buckets,
        series=[Series.unmarshal(s) for s in series],
    )

    data = {
        "status": status,
        "snap_id": "test-id",
        "metric_name": str(MetricName.DAILY_DEVICE_CHANGE),
        "buckets": buckets,
        "series": series,
    }

    assert Metric.unmarshal(data) == results


@pytest.mark.parametrize("status", [False, {1}, 1, "INVALID"])
def test_metric_results_invalid_status(status):
    metric_results = {
        "status": status,
        "snap_id": "test-id",
        "metric_name": "test-metric",
        "buckets": [],
        "series": [],
    }

    with pytest.raises(ValidationError):
        Metric.unmarshal(metric_results)


@pytest.mark.parametrize("snap_id", [False, {1}, 1])
def test_metric_results_invalid_snap_id(snap_id):
    metric_results = {
        "status": "OK",
        "snap_id": snap_id,
        "metric_name": "test-metric",
        "buckets": [],
        "series": [],
    }

    with pytest.raises(ValidationError):
        Metric.unmarshal(metric_results)


@pytest.mark.parametrize("metric_name", [None, False, {1}, 1])
def test_metric_results_invalid_metric_name(metric_name):
    metric_results = {
        "status": "OK",
        "snap_id": "test-id",
        "metric_name": metric_name,
        "buckets": [],
        "series": [],
    }

    with pytest.raises(ValidationError):
        Metric.unmarshal(metric_results)


@pytest.mark.parametrize("buckets", [None, False, {1}, 1])
def test_metric_results_invalid_buckets(buckets):
    metric_results = {
        "status": "OK",
        "snap_id": "test-id",
        "metric_name": "test-metric",
        "buckets": buckets,
        "series": [],
    }

    with pytest.raises(ValidationError):
        Metric.unmarshal(metric_results)


@pytest.mark.parametrize("series", [None, False, {1}, 1])
def test_metric_results_invalid_series(series):
    metric_results = {
        "status": "OK",
        "snap_id": "test-id",
        "metric_name": "test-metric",
        "buckets": [],
        "series": series,
    }

    with pytest.raises(ValidationError):
        Metric.unmarshal(metric_results)


def test_metrics_results_unmarshal_no_data():
    data = {
        "metrics": [
            {
                "buckets": [],
                "metric_name": "weekly_installed_base_by_channel",
                "series": [],
                "snap_id": "test-snap-id",
                "status": "NO_DATA",
            }
        ]
    }

    metrics_results = MetricsResponse.unmarshal(data)

    assert metrics_results == MetricsResponse(
        metrics=[
            Metric(
                status="NO_DATA",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_INSTALLED_BASE_BY_CHANNEL,
                buckets=[],
                series=[],
            )
        ]
    )


@pytest.mark.parametrize("metric_name", MetricName)
def test_metrics_results_unmarshal_one_bucket(metric_name):
    data = {
        "metrics": [
            {
                "buckets": ["2021-01-01"],
                "metric_name": metric_name,
                "series": [
                    {"name": "continued", "values": [11]},
                    {"name": "lost", "values": [3]},
                    {"name": "new", "values": [4]},
                ],
                "snap_id": "test-snap-id",
                "status": "OK",
            }
        ]
    }

    metrics_results = MetricsResponse.unmarshal(data)

    assert metrics_results == MetricsResponse(
        metrics=[
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=metric_name,
                buckets=["2021-01-01"],
                series=[
                    Series(name="continued", values=[11], currently_released=None),
                    Series(name="lost", values=[3], currently_released=None),
                    Series(name="new", values=[4], currently_released=None),
                ],
            )
        ]
    )


def test_metrics_results_unmarshal_two_buckets():
    data = {
        "metrics": [
            {
                "buckets": ["2021-01-01", "2021-01-02"],
                "metric_name": "daily_device_change",
                "series": [
                    {"name": "continued", "values": [11, 15]},
                    {"name": "lost", "values": [3, 0]},
                    {"name": "new", "values": [4, 1]},
                ],
                "snap_id": "test-snap-id",
                "status": "OK",
            }
        ]
    }

    metrics_results = MetricsResponse.unmarshal(data)

    assert metrics_results == MetricsResponse(
        metrics=[
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.DAILY_DEVICE_CHANGE,
                buckets=["2021-01-01", "2021-01-02"],
                series=[
                    Series(name="continued", values=[11, 15], currently_released=None),
                    Series(name="lost", values=[3, 0], currently_released=None),
                    Series(name="new", values=[4, 1], currently_released=None),
                ],
            )
        ]
    )


def test_metrics_results_unmarshal_with_currently_released():
    data = {
        "metrics": [
            {
                "buckets": ["2021-01-01"],
                "metric_name": "weekly_installed_base_by_channel",
                "series": [
                    {"currently_released": False, "name": "beta", "values": [62]},
                    {"currently_released": True, "name": "edge", "values": [38]},
                ],
                "snap_id": "test-snap-id",
                "status": "OK",
            }
        ]
    }

    metrics_results = MetricsResponse.unmarshal(data)

    assert metrics_results == MetricsResponse(
        metrics=[
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_INSTALLED_BASE_BY_CHANNEL,
                buckets=["2021-01-01"],
                series=[
                    Series(name="beta", values=[62], currently_released=False),
                    Series(name="edge", values=[38], currently_released=True),
                ],
            )
        ]
    )


def test_metrics_results_unmarshal_multiple_metrics():
    metrics = [
        {
            "buckets": ["2021-01-01"],
            "metric_name": str(n),
            "series": [
                {"name": "blah", "values": [1]},
                {"name": "blahh", "values": [1, 2]},
                {"name": "blahhh", "values": [1, 2, 3]},
            ],
            "snap_id": "test-snap-id",
            "status": "OK",
        }
        for n in MetricName
    ]
    data = {"metrics": metrics}

    metrics_results = MetricsResponse.unmarshal(data)

    assert metrics_results == MetricsResponse(
        metrics=[
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.DAILY_DEVICE_CHANGE,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.INSTALLED_BASE_BY_ARCHITECTURE,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.INSTALLED_BASE_BY_CHANNEL,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.INSTALLED_BASE_BY_COUNTRY,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.INSTALLED_BASE_BY_OPERATING_SYSTEM,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.INSTALLED_BASE_BY_VERSION,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_DEVICE_CHANGE,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_INSTALLED_BASE_BY_ARCHITECTURE,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_INSTALLED_BASE_BY_CHANNEL,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_INSTALLED_BASE_BY_COUNTRY,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
            Metric(
                status="OK",
                snap_id="test-snap-id",
                metric_name=MetricName.WEEKLY_INSTALLED_BASE_BY_VERSION,
                buckets=["2021-01-01"],
                series=[
                    Series(name="blah", values=[1], currently_released=None),
                    Series(name="blahh", values=[1, 2], currently_released=None),
                    Series(name="blahhh", values=[1, 2, 3], currently_released=None),
                ],
            ),
        ]
    )


@pytest.mark.parametrize("results", [False, {1}, 1, "INVALID", []])
def test_metrics_results_invalid_payload(results):
    with pytest.raises(TypeError):
        MetricsResponse.unmarshal(results)


@pytest.mark.parametrize("metrics", [False, {1}, 1, "INVALID", {}])
def test_metrics_metrics_invalid_metrics(metrics):
    data = {"metrics": metrics}
    with pytest.raises(ValidationError):
        MetricsResponse.unmarshal(data)


@pytest.mark.parametrize(
    ("metric_name", "expected_label"),
    [
        (MetricName.DAILY_DEVICE_CHANGE, "Devices"),
        (MetricName.WEEKLY_DEVICE_CHANGE, "Devices"),
        (MetricName.INSTALLED_BASE_BY_CHANNEL, "Channel"),
        (MetricName.WEEKLY_INSTALLED_BASE_BY_CHANNEL, "Channel"),
        (MetricName.INSTALLED_BASE_BY_COUNTRY, "Country"),
        (MetricName.WEEKLY_INSTALLED_BASE_BY_COUNTRY, "Country"),
        (MetricName.INSTALLED_BASE_BY_OPERATING_SYSTEM, "OS"),
        (MetricName.WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM, "OS"),
        (MetricName.INSTALLED_BASE_BY_VERSION, "Version"),
        (MetricName.WEEKLY_INSTALLED_BASE_BY_VERSION, "Version"),
        (MetricName.INSTALLED_BASE_BY_ARCHITECTURE, "Architecture"),
        (MetricName.WEEKLY_INSTALLED_BASE_BY_ARCHITECTURE, "Architecture"),
    ],
)
def test_to_label(metric_name: MetricName, expected_label: str) -> None:
    assert metric_name.to_label() == expected_label


@pytest.mark.parametrize(
    ("daily", "weekly"),
    [
        (MetricName.DAILY_DEVICE_CHANGE, MetricName.WEEKLY_DEVICE_CHANGE),
        (
            MetricName.INSTALLED_BASE_BY_CHANNEL,
            MetricName.WEEKLY_INSTALLED_BASE_BY_CHANNEL,
        ),
        (
            MetricName.INSTALLED_BASE_BY_COUNTRY,
            MetricName.WEEKLY_INSTALLED_BASE_BY_COUNTRY,
        ),
        (
            MetricName.INSTALLED_BASE_BY_OPERATING_SYSTEM,
            MetricName.WEEKLY_INSTALLED_BASE_BY_OPERATING_SYSTEM,
        ),
        (
            MetricName.INSTALLED_BASE_BY_VERSION,
            MetricName.WEEKLY_INSTALLED_BASE_BY_VERSION,
        ),
        (
            MetricName.INSTALLED_BASE_BY_ARCHITECTURE,
            MetricName.WEEKLY_INSTALLED_BASE_BY_ARCHITECTURE,
        ),
    ],
)
def test_to_label_daily_weekly_parity(daily: MetricName, weekly: MetricName) -> None:
    """Daily and weekly variants of the same metric share the same label."""
    assert daily.to_label() == weekly.to_label()
